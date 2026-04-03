#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <float.h>
#include <errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "cJSON.h"

#include "esp_adc/adc_oneshot.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "led_strip.h"

#define BATT_ADC_UNIT             ADC_UNIT_1
#define BATT_ADC_CHANNEL          ADC_CHANNEL_6   // GPIO7
#define BATT_ADC_ATTEN            ADC_ATTEN_DB_12

#define SPI_HOST_USED             SPI2_HOST
#define SPI_PIN_MOSI              35
#define SPI_PIN_SCLK              36
#define SPI_PIN_MISO              37
#define SPI_PIN_CS                38

#define WS2812_GPIO               48

#define UDP_TX_PORT               19000
#define UDP_RX_PORT               19001

#define HEARTBEAT_BROADCAST_MS    1000
#define SAMPLE_PERIOD_MS          10
#define CLIENT_TIMEOUT_MS         5000
#define LOW_BATT_THRESHOLD_V      3.5f
#define LED_BLINK_PERIOD_MS       1000
#define LED_BLINK_ON_MS           500
#define LED_TX_PULSE_MS           15

#define WIFI_CONNECTED_BIT        BIT0

#define DEVICE_NAME               "DAS_NODE_CM01"

// 调试开关：1=不依赖真实ADC/ADS硬件，使用模拟数据
#define ADC_SIM_MODE              0

#define BATCH_SIZE                8
#define ADC_CHANNEL_COUNT         3

// ADS1118: PGA=±4.096V -> LSB=125uV
#define ADS1118_LSB_V             0.000125f
#define ADS_AIN1_REF_V            1.25f
#define ADS_AIN0_DIV_RATIO        ((1000.0f + 39.0f) / 39.0f)
#define ACS725_ZERO_V             (3.3f / 2.0f)
#define ACS725_SENSITIVITY_V_PER_A 0.0264f
#define CALIB_VOLTAGE_NEAR_ZERO_TH 0.3f
#define CALIB_HOLD_MS             2000
#define CALIB_STARTUP_WINDOW_MS   10000
#define CALIB_MAX_SAMPLES         ((CALIB_HOLD_MS / SAMPLE_PERIOD_MS) + 8)

static const char *TAG = "DAS_NODE_CM01";

static adc_oneshot_unit_handle_t s_adc_handle;
static spi_device_handle_t s_ads1118_dev;
static led_strip_handle_t s_led_strip;
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t s_client_mutex;

typedef enum {
    ADC_CH_BATT = 0,
    ADC_CH_ADS_VOLTAGE,
    ADC_CH_ADS_CURRENT,
} adc_channel_id_t;

typedef struct {
    const char *channel;
    int64_t ts_ms;
    float current;
    float max8;
    float min8;
} sample_record_t;

typedef struct {
    sample_record_t rec[ADC_CHANNEL_COUNT];
} sample_frame_t;

typedef struct {
    bool connected;
    struct sockaddr_in addr;
    int64_t last_keepalive_ms;
} client_state_t;

static client_state_t s_client_state = {0};
static int s_udp_sock = -1;

// 采样发送任务使用的静态缓存，避免大对象占用任务栈导致栈溢出
static char s_tx_buf[3072];
static sample_frame_t s_batch[BATCH_SIZE];
static float s_hist[ADC_CHANNEL_COUNT][BATCH_SIZE];

typedef enum {
    LED_STATE_WIFI_DOWN = 0,
    LED_STATE_WIFI_UP_NO_APP,
    LED_STATE_APP_CONNECTED,
    LED_STATE_LOW_BATT,
} led_state_t;

static bool client_is_connected(void);
static int64_t now_ms(void);
static bool is_valid_heartbeat_json(const char *payload);

static volatile bool s_low_power_mode = false;
static volatile int64_t s_last_tx_flash_ms = 0;
static volatile float s_batt_voltage_cache = 0.0f;

static volatile int16_t s_ads_raw_diff = 0;
static volatile int16_t s_ads_raw_ain2 = 0;
static volatile int16_t s_ads_raw_ain3 = 0;
static volatile float s_ads_v_diff = 0.0f;
static volatile float s_ads_v_ain2 = 0.0f;
static volatile float s_ads_v_ain3 = 0.0f;
static volatile float s_ads_voltage_uncal = 0.0f;
static volatile float s_ads_current_uncal = 0.0f;

static bool s_ads_last_mux_valid = false;
static uint8_t s_ads_last_mux = 0xFF;
static uint8_t s_ads_next_mux = 0;

static float read_batt_voltage_once(void)
{
    int batt_raw = 0;
    if (adc_oneshot_read(s_adc_handle, BATT_ADC_CHANNEL, &batt_raw) == ESP_OK) {
        float v_adc = (batt_raw / 4095.0f) * 3.3f;
        return v_adc * 2.226f;
    }
    return 0.0f;
}

static void enter_low_power_mode(float batt_v)
{
    if (s_low_power_mode) {
        return;
    }

    s_low_power_mode = true;
    ESP_LOGW(TAG, "LOW_BATT detected: %.3fV, entering low power mode", batt_v);

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    esp_wifi_disconnect();
    esp_wifi_stop();

    if (s_udp_sock >= 0) {
        shutdown(s_udp_sock, 0);
        close(s_udp_sock);
        s_udp_sock = -1;
    }

    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    memset(&s_client_state, 0, sizeof(s_client_state));
    xSemaphoreGive(s_client_mutex);

    ESP_LOGW(TAG, "WiFi stopped, UDP closed, TX cache will be cleared in sampling task");
}

static void init_batt_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BATT_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, BATT_ADC_CHANNEL, &chan_config));
}

static void init_ads1118_spi(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_PIN_MOSI,
        .miso_io_num = SPI_PIN_MISO,
        .sclk_io_num = SPI_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &devcfg, &s_ads1118_dev));

    gpio_config_t cs_io_conf = {
        .pin_bit_mask = 1ULL << SPI_PIN_CS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_io_conf));

    // 测试模式：CS 持续保持拉低
    ESP_ERROR_CHECK(gpio_set_level(SPI_PIN_CS, 0));
}

static void ads1118_transfer_word(uint16_t tx_word, uint16_t *rx_prev_raw, uint16_t *rx_cfg_echo)
{
    uint8_t tx_data[4] = {
        (uint8_t)(tx_word >> 8),
        (uint8_t)(tx_word & 0xFF),
        0x00,
        0x00,
    };
    uint8_t rx_data[4] = {0};

    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(s_ads1118_dev, &t));

    if (rx_prev_raw) {
        *rx_prev_raw = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    }
    if (rx_cfg_echo) {
        *rx_cfg_echo = (uint16_t)((rx_data[2] << 8) | rx_data[3]);
    }
}

static uint16_t ads1118_build_config(uint8_t mux)
{
    enum {
        ADS1118_CFG_OS_SINGLE        = 1u << 15,
        ADS1118_CFG_PGA_4V096        = 1u << 9,
        ADS1118_CFG_MODE_SINGLE_SHOT = 1u << 8,
        ADS1118_CFG_DR_475SPS        = 6u << 5,
        ADS1118_CFG_TS_MODE_ADC      = 0u << 4,
        ADS1118_CFG_PULLUP_EN        = 1u << 3,
        ADS1118_CFG_NOP_VALID        = 1u << 1,
        ADS1118_CFG_RESERVED         = 1u,
    };

    return (uint16_t)(ADS1118_CFG_OS_SINGLE |
                      ((uint16_t)(mux & 0x07u) << 12) |
                      ADS1118_CFG_PGA_4V096 |
                      ADS1118_CFG_MODE_SINGLE_SHOT |
                      ADS1118_CFG_DR_475SPS |
                      ADS1118_CFG_TS_MODE_ADC |
                      ADS1118_CFG_PULLUP_EN |
                      ADS1118_CFG_NOP_VALID |
                      ADS1118_CFG_RESERVED);
}

static void ads1118_sample_step(void)
{
    uint16_t prev_raw = 0;
    uint16_t cfg_echo = 0;
    uint8_t mux = s_ads_next_mux;
    uint16_t cfg = ads1118_build_config(mux);

    ads1118_transfer_word(cfg, &prev_raw, &cfg_echo);

    if (s_ads_last_mux_valid) {
        switch (s_ads_last_mux) {
        case 0:
            s_ads_raw_diff = (int16_t)prev_raw;
            break;
        case 6:
            s_ads_raw_ain2 = (int16_t)prev_raw;
            break;
        case 7:
            s_ads_raw_ain3 = (int16_t)prev_raw;
            break;
        default:
            break;
        }
    }

    s_ads_last_mux = mux;
    s_ads_last_mux_valid = true;

    switch (mux) {
    case 0:
        s_ads_next_mux = 6;
        break;
    case 6:
        s_ads_next_mux = 7;
        break;
    case 7:
    default:
        s_ads_next_mux = 0;
        break;
    }

    (void)cfg_echo;
}

static const char *channel_name(adc_channel_id_t ch)
{
    switch (ch) {
    case ADC_CH_BATT: return "BATT_ADC";
    case ADC_CH_ADS_VOLTAGE: return "ADS_VOLTAGE";
    case ADC_CH_ADS_CURRENT: return "ADS_CURRENT";
    default: return "UNKNOWN";
    }
}

static void read_all_channels_values(float out_values[ADC_CHANNEL_COUNT])
{
#if ADC_SIM_MODE
    static uint32_t t = 0;
    t++;
    // 简单平滑模拟：电池、电压通道、电流通道
    out_values[ADC_CH_BATT] = 22.0f + ((float)(t % 120) * 0.05f);     // 22.0 ~ 28.0V
    out_values[ADC_CH_ADS_VOLTAGE] = 16.0f + ((float)((t * 3) % 200) * 0.1f); // 16.0 ~ 36.0V
    out_values[ADC_CH_ADS_CURRENT] = -40.0f + ((float)((t * 5) % 800) * 0.1f); // -40.0 ~ 40.0A

    s_ads_raw_diff = 0;
    s_ads_raw_ain2 = 0;
    s_ads_raw_ain3 = 0;
    s_ads_v_diff = 0.0f;
    s_ads_v_ain2 = 0.0f;
    s_ads_v_ain3 = 0.0f;
    s_ads_voltage_uncal = out_values[ADC_CH_ADS_VOLTAGE];
    s_ads_current_uncal = out_values[ADC_CH_ADS_CURRENT];
#else
    out_values[ADC_CH_BATT] = s_batt_voltage_cache;

    ads1118_sample_step();

    int16_t raw_diff = s_ads_raw_diff;
    int16_t raw_ain2 = s_ads_raw_ain2;
    int16_t raw_ain3 = s_ads_raw_ain3;

    float v_diff = (float)raw_diff * ADS1118_LSB_V;
    float v_ain2 = (float)raw_ain2 * ADS1118_LSB_V;
    float v_ain3 = (float)raw_ain3 * ADS1118_LSB_V;
    float v_ain0 = v_diff + ADS_AIN1_REF_V;
    float voltage = v_ain0 * ADS_AIN0_DIV_RATIO;

    float i_ain2 = (v_ain2 - ACS725_ZERO_V) / ACS725_SENSITIVITY_V_PER_A;
    float i_ain3 = (v_ain3 - ACS725_ZERO_V) / ACS725_SENSITIVITY_V_PER_A;
    float current = i_ain2 + i_ain3;

    s_ads_raw_diff = raw_diff;
    s_ads_raw_ain2 = raw_ain2;
    s_ads_raw_ain3 = raw_ain3;
    s_ads_v_diff = v_diff;
    s_ads_v_ain2 = v_ain2;
    s_ads_v_ain3 = v_ain3;
    s_ads_voltage_uncal = voltage;
    s_ads_current_uncal = current;

    out_values[ADC_CH_ADS_VOLTAGE] = voltage;
    out_values[ADC_CH_ADS_CURRENT] = current;
#endif
}

static void compute_window_stats(const float hist[ADC_CHANNEL_COUNT][BATCH_SIZE], int count,
                                 float out_min[ADC_CHANNEL_COUNT], float out_max[ADC_CHANNEL_COUNT])
{
    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;
        for (int i = 0; i < count; i++) {
            float v = hist[ch][i];
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
        out_min[ch] = (count > 0) ? min_v : 0.0f;
        out_max[ch] = (count > 0) ? max_v : 0.0f;
    }
}

static void sort_float_array(float *arr, int count)
{
    for (int i = 1; i < count; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

static float compute_median(const float *values, int count)
{
    if (count <= 0) {
        return 0.0f;
    }

    float sorted[CALIB_MAX_SAMPLES];
    int capped_count = (count > CALIB_MAX_SAMPLES) ? CALIB_MAX_SAMPLES : count;
    memcpy(sorted, values, (size_t)capped_count * sizeof(float));
    sort_float_array(sorted, capped_count);

    if ((capped_count & 1) != 0) {
        return sorted[capped_count / 2];
    }

    int upper = capped_count / 2;
    int lower = upper - 1;
    return (sorted[lower] + sorted[upper]) * 0.5f;
}

static void init_ws2812(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_led_strip));
}

static bool wifi_is_connected(void)
{
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static led_state_t get_led_state(void)
{
    if (s_low_power_mode) {
        return LED_STATE_LOW_BATT;
    }
    if (!wifi_is_connected()) {
        return LED_STATE_WIFI_DOWN;
    }
    if (client_is_connected()) {
        return LED_STATE_APP_CONNECTED;
    }
    return LED_STATE_WIFI_UP_NO_APP;
}

static void task_status_led(void *arg)
{
    (void)arg;

    while (1) {
        led_state_t state = get_led_state();
        int64_t now = now_ms();
        bool blink_on = ((now % LED_BLINK_PERIOD_MS) < LED_BLINK_ON_MS);

        uint8_t r = 0, g = 0, b = 0;
        switch (state) {
        case LED_STATE_WIFI_DOWN:
            // WiFi 未连接：黄灯闪烁
            if (blink_on) {
                r = 32; g = 16; b = 0;
            }
            break;
        case LED_STATE_WIFI_UP_NO_APP:
            // WiFi 已连接但未连接 APP：绿灯闪烁
            if (blink_on) {
                r = 0; g = 32; b = 0;
            }
            break;
        case LED_STATE_APP_CONNECTED: {
            // APP 已连接：绿灯常亮；UDP 发送时短脉冲闪烁（类似网口灯）
            bool flash_active = (now - s_last_tx_flash_ms) <= LED_TX_PULSE_MS;
            if (flash_active) {
                r = 0; g = 0; b = 0;
            } else {
                r = 0; g = 32; b = 0;
            }
            break;
        }
        case LED_STATE_LOW_BATT:
            // 低电压：红灯每秒短闪一次（80ms on + 920ms off）
            if ((now % 1000) < 80) {
                r = 32; g = 0; b = 0;
            }
            break;
        default:
            break;
        }

        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void init_wifi_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
        },
    };
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void init_udp_socket(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ESP_ERROR_CHECK(s_udp_sock < 0 ? ESP_FAIL : ESP_OK);

    int broadcast_enable = 1;
    ESP_ERROR_CHECK(setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST,
                               &broadcast_enable, sizeof(broadcast_enable)) < 0 ? ESP_FAIL : ESP_OK);

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_RX_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    ESP_ERROR_CHECK(bind(s_udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0 ? ESP_FAIL : ESP_OK);

    struct timeval timeout = {
        .tv_sec = 1,
        .tv_usec = 0,
    };
    ESP_ERROR_CHECK(setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0 ? ESP_FAIL : ESP_OK);
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool client_is_connected(void)
{
    bool connected;
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    connected = s_client_state.connected && ((now_ms() - s_client_state.last_keepalive_ms) <= CLIENT_TIMEOUT_MS);
    if (s_client_state.connected && !connected) {
        s_client_state.connected = false;
    }
    xSemaphoreGive(s_client_mutex);
    return connected;
}

static bool is_valid_heartbeat_json(const char *payload)
{
    if (payload == NULL) {
        return false;
    }

    cJSON *root = cJSON_Parse(payload);
    if (root == NULL) {
        return false;
    }

    cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    bool valid = cJSON_IsString(type) && (type->valuestring != NULL) &&
                 (strcmp(type->valuestring, "heartbeat") == 0);

    cJSON_Delete(root);
    return valid;
}

static void task_net_rx(void *arg)
{
    (void)arg;
    char rx_buf[128];

    while (1) {
        if (s_low_power_mode || s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        struct sockaddr_in src_addr = {0};
        socklen_t addr_len = sizeof(src_addr);
        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&src_addr, &addr_len);
        if (len > 0) {
            rx_buf[len] = '\0';
            if (!is_valid_heartbeat_json(rx_buf)) {
                continue;
            }

            char ip_str[16] = {0};
            inet_ntoa_r(src_addr.sin_addr, ip_str, sizeof(ip_str));
            xSemaphoreTake(s_client_mutex, portMAX_DELAY);
            s_client_state.addr = src_addr;
            s_client_state.last_keepalive_ms = now_ms();
            if (!s_client_state.connected) {
                ESP_LOGI(TAG, "Client connected: %s:%u", ip_str, ntohs(src_addr.sin_port));
            }
            s_client_state.connected = true;
            xSemaphoreGive(s_client_mutex);
        } else if (len == 0) {
            // UDP上几乎不会出现0长度负载，忽略即可
            continue;
        } else {
            int err = errno;
            if (err == EAGAIN || err == EWOULDBLOCK) {
                // recv超时，正常轮询
                continue;
            }
            ESP_LOGW(TAG, "recvfrom failed, errno=%d", err);
        }
    }
}

static void task_heartbeat(void *arg)
{
    (void)arg;
    const char *msg = "{\"type\":\"esp_heartbeat\",\"dev\":\"" DEVICE_NAME "\"}";

    struct sockaddr_in bcast_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_TX_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    while (1) {
        if (s_low_power_mode || s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_BROADCAST_MS));
            continue;
        }

        if (!client_is_connected()) {
            int ret = sendto(s_udp_sock, msg, strlen(msg), 0,
                             (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
            if (ret < 0) {
                ESP_LOGW(TAG, "heartbeat send failed, errno=%d", errno);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_BROADCAST_MS));
    }
}

static void task_sampling_tx(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t seq = 0;
    uint32_t log_div = 0;
    uint32_t stack_log_div = 0;
    int batch_count = 0;
    int hist_count = 0;
    int hist_write_idx = 0;
    bool cache_cleared_when_disconnected = false;
    int64_t last_batt_sample_ms = 0;
    int64_t boot_ms = now_ms();
    int64_t last_ads_diag_log_ms = 0;
    bool calib_done = false;
    bool calib_decided = false;
    bool calib_hold_active = false;
    int64_t calib_hold_start_ms = 0;
    float calib_samples_v[CALIB_MAX_SAMPLES] = {0};
    float calib_samples_i[CALIB_MAX_SAMPLES] = {0};
    int calib_count = 0;
    float voltage_offset = 0.0f;
    float current_offset = 0.0f;

    memset(s_batch, 0, sizeof(s_batch));
    memset(s_hist, 0, sizeof(s_hist));
    memset(s_tx_buf, 0, sizeof(s_tx_buf));

#if !ADC_SIM_MODE
    s_batt_voltage_cache = read_batt_voltage_once();
    last_batt_sample_ms = now_ms();
    ESP_LOGI(TAG, "BATT 1Hz: %.3fV", s_batt_voltage_cache);
    if (s_batt_voltage_cache < LOW_BATT_THRESHOLD_V) {
        enter_low_power_mode(s_batt_voltage_cache);
    }
#endif

    while (1) {
        float values[ADC_CHANNEL_COUNT] = {0};
        int64_t ts_ms = now_ms();

#if !ADC_SIM_MODE
        if ((ts_ms - last_batt_sample_ms) >= 1000) {
            s_batt_voltage_cache = read_batt_voltage_once();
            last_batt_sample_ms = ts_ms;
            ESP_LOGI(TAG, "BATT 1Hz: %.3fV", s_batt_voltage_cache);

            // 非 SIM 模式下，与连接状态无关：只要低电就进入低电状态
            if (s_batt_voltage_cache < LOW_BATT_THRESHOLD_V) {
                enter_low_power_mode(s_batt_voltage_cache);
            }
        }
#endif

        read_all_channels_values(values);

        if (!calib_decided) {
            int64_t startup_elapsed = ts_ms - boot_ms;
            float v_uncal = s_ads_voltage_uncal;
            float i_uncal = s_ads_current_uncal;

            if (startup_elapsed <= CALIB_STARTUP_WINDOW_MS) {
                bool near_zero = (v_uncal > -CALIB_VOLTAGE_NEAR_ZERO_TH) && (v_uncal < CALIB_VOLTAGE_NEAR_ZERO_TH);
                if (near_zero) {
                    if (!calib_hold_active) {
                        calib_hold_active = true;
                        calib_hold_start_ms = ts_ms;
                        calib_count = 0;
                    }

                    if (calib_count < CALIB_MAX_SAMPLES) {
                        calib_samples_v[calib_count] = v_uncal;
                        calib_samples_i[calib_count] = i_uncal;
                        calib_count++;
                    }

                    if ((ts_ms - calib_hold_start_ms) >= CALIB_HOLD_MS && calib_count > 0) {
                        voltage_offset = compute_median(calib_samples_v, calib_count);
                        current_offset = compute_median(calib_samples_i, calib_count);
                        calib_done = true;
                        calib_decided = true;
                        ESP_LOGI(TAG, "Startup calibration done (median): v_off=%.5f, i_off=%.5f, samples=%d",
                                 voltage_offset, current_offset, calib_count);
                    }
                } else {
                    calib_hold_active = false;
                    calib_count = 0;
                }
            } else {
                calib_decided = true;
                ESP_LOGW(TAG, "Startup calibration skipped: no continuous |V|<%.3fV for %dms within %dms window",
                         CALIB_VOLTAGE_NEAR_ZERO_TH, CALIB_HOLD_MS, CALIB_STARTUP_WINDOW_MS);
            }
        }

        if (calib_done) {
            values[ADC_CH_ADS_VOLTAGE] -= voltage_offset;
            values[ADC_CH_ADS_CURRENT] -= current_offset;
        }

        if ((ts_ms - last_ads_diag_log_ms) >= 1000) {
            last_ads_diag_log_ms = ts_ms;
            ESP_LOGI(TAG,
                     "ADS diag raw: diff=0x%04X(%d) ain2=0x%04X(%d) ain3=0x%04X(%d), vdiff=%.5f vain2=%.5f vain3=%.5f, out_v=%.5f out_i=%.5f, calib=%s",
                     (uint16_t)s_ads_raw_diff, (int)s_ads_raw_diff,
                     (uint16_t)s_ads_raw_ain2, (int)s_ads_raw_ain2,
                     (uint16_t)s_ads_raw_ain3, (int)s_ads_raw_ain3,
                     s_ads_v_diff, s_ads_v_ain2, s_ads_v_ain3,
                     values[ADC_CH_ADS_VOLTAGE], values[ADC_CH_ADS_CURRENT],
                     calib_done ? "ON" : "OFF");
        }

        if (s_low_power_mode) {
            memset(s_batch, 0, sizeof(s_batch));
            memset(s_hist, 0, sizeof(s_hist));
            batch_count = 0;
            hist_count = 0;
            hist_write_idx = 0;
            seq = 0;
            cache_cleared_when_disconnected = true;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // 更新最近8点历史
        for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            s_hist[ch][hist_write_idx] = values[ch];
        }
        hist_write_idx = (hist_write_idx + 1) % BATCH_SIZE;
        if (hist_count < BATCH_SIZE) {
            hist_count++;
        }

        float min8[ADC_CHANNEL_COUNT] = {0};
        float max8[ADC_CHANNEL_COUNT] = {0};
        compute_window_stats(s_hist, hist_count, min8, max8);

        bool connected = client_is_connected();
        if (!connected && !cache_cleared_when_disconnected) {
            memset(s_batch, 0, sizeof(s_batch));
            memset(s_hist, 0, sizeof(s_hist));
            batch_count = 0;
            hist_count = 0;
            hist_write_idx = 0;
            seq = 0;
            cache_cleared_when_disconnected = true;
            ESP_LOGI(TAG, "Client disconnected, TX cache cleared, seq reset");
        }
        if (connected && cache_cleared_when_disconnected) {
            cache_cleared_when_disconnected = false;
            ESP_LOGI(TAG, "Client reconnected, resume TX with fresh samples");
        }

        // 存入本次3通道记录
        for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            s_batch[batch_count].rec[ch].channel = channel_name((adc_channel_id_t)ch);
            s_batch[batch_count].rec[ch].ts_ms = ts_ms;
            s_batch[batch_count].rec[ch].current = values[ch];
            s_batch[batch_count].rec[ch].max8 = max8[ch];
            s_batch[batch_count].rec[ch].min8 = min8[ch];
        }
        batch_count++;

        if (batch_count >= BATCH_SIZE) {
            if (connected) {
                int off = snprintf(s_tx_buf, sizeof(s_tx_buf),
                                   "{\"type\":\"data_batch\",\"seq\":%" PRIu32 ",\"records\":[",
                                   seq++);
                if (off > 0 && off < (int)sizeof(s_tx_buf)) {
                    bool first = true;
                    for (int i = 0; i < BATCH_SIZE; i++) {
                        for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
                            sample_record_t *r = &s_batch[i].rec[ch];
                            int n = snprintf(s_tx_buf + off, sizeof(s_tx_buf) - (size_t)off,
                                             "%s{\"ch\":\"%s\",\"ts\":%" PRIi64
                                             ",\"max8\":%.4f,\"cur\":%.4f,\"min8\":%.4f}",
                                             first ? "" : ",", r->channel, r->ts_ms,
                                             r->max8, r->current, r->min8);
                            if (n <= 0 || (off + n) >= (int)sizeof(s_tx_buf)) {
                                off = -1;
                                break;
                            }
                            off += n;
                            first = false;
                        }
                        if (off < 0) {
                            break;
                        }
                    }
                    if (off > 0 && (off + 3) < (int)sizeof(s_tx_buf)) {
                        s_tx_buf[off++] = ']';
                        s_tx_buf[off++] = '}';
                        s_tx_buf[off] = '\0';

                        struct sockaddr_in dst = {0};
                        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
                        dst = s_client_state.addr;
                        xSemaphoreGive(s_client_mutex);

                        int ret = sendto(s_udp_sock, s_tx_buf, (size_t)off, 0,
                                         (struct sockaddr *)&dst, sizeof(dst));
                        if (ret < 0) {
                            ESP_LOGW(TAG, "data batch send failed, errno=%d", errno);
                        } else {
                            s_last_tx_flash_ms = now_ms();
                        }
                    }

                    if (++log_div >= 10) {
                        log_div = 0;
                        ESP_LOGI(TAG, "TX batch seq=%" PRIu32 " rec=24 batt=%.3f voltage=%.3f current=%.3f",
                                 seq, values[ADC_CH_BATT], values[ADC_CH_ADS_VOLTAGE], values[ADC_CH_ADS_CURRENT]);
                    }

                    if (++stack_log_div >= 50) {
                        stack_log_div = 0;
                        UBaseType_t wm_words = uxTaskGetStackHighWaterMark(NULL);
                        size_t wm_bytes = ((size_t)wm_words) * sizeof(StackType_t);
                        ESP_LOGI(TAG, "sampling_tx stack high water mark: %lu words (~%lu bytes)",
                                 (unsigned long)wm_words,
                                 (unsigned long)wm_bytes);
                    }
                }
            }

            batch_count = 0;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot: ADC(GPIO7), SPI(CS38/MISO37/SCLK36/MOSI35), WS2812(GPIO48), UDP tx=%d rx=%d",
             UDP_TX_PORT, UDP_RX_PORT);
    ESP_LOGI(TAG, "WiFi config: ssid=%s, pass=%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    s_client_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((s_wifi_event_group && s_client_mutex) ? ESP_OK : ESP_FAIL);

    if (!ADC_SIM_MODE) {
        init_batt_adc();
        init_ads1118_spi();
    } else {
        ESP_LOGW(TAG, "ADC_SIM_MODE enabled: using simulated ADC data");
    }
    init_ws2812();
    init_wifi_sta();

    xTaskCreate(task_status_led, "status_led", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "Waiting WiFi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    init_udp_socket();

    xTaskCreate(task_net_rx, "net_rx", 4096, NULL, 6, NULL);
    xTaskCreate(task_heartbeat, "heartbeat", 3072, NULL, 4, NULL);
    xTaskCreate(task_sampling_tx, "sampling_tx", 12288, NULL, 5, NULL);
}
