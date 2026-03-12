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

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "esp_adc/adc_oneshot.h"

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

#define UDP_PORT                  9000

#define HEARTBEAT_BROADCAST_MS    1000
#define SAMPLE_PERIOD_MS          10
#define CLIENT_TIMEOUT_MS         5000

#define WIFI_CONNECTED_BIT        BIT0

#define DEVICE_NAME               "DAS_NODE_CM01"

static const char *TAG = "DAS_NODE_CM01";

static adc_oneshot_unit_handle_t s_adc_handle;
static spi_device_handle_t s_ads1118_dev;
static led_strip_handle_t s_led_strip;
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t s_client_mutex;

typedef struct {
    bool connected;
    struct sockaddr_in addr;
    int64_t last_keepalive_ms;
} client_state_t;

static client_state_t s_client_state = {0};
static int s_udp_sock = -1;

typedef enum {
    LED_STATE_WIFI_DOWN = 0,
    LED_STATE_WIFI_UP_NO_APP,
    LED_STATE_APP_CONNECTED,
} led_state_t;

static bool client_is_connected(void);

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
        .spics_io_num = SPI_PIN_CS,
        .queue_size = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &devcfg, &s_ads1118_dev));
}

static uint16_t ads1118_transfer_word(uint16_t tx_word)
{
    uint8_t tx_data[2] = {(uint8_t)(tx_word >> 8), (uint8_t)(tx_word & 0xFF)};
    uint8_t rx_data[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(s_ads1118_dev, &t));

    return (uint16_t)((rx_data[0] << 8) | rx_data[1]);
}

static int16_t ads1118_read_raw(void)
{
    // 写配置并读回上一拍转换结果：示例配置
    uint16_t raw = ads1118_transfer_word(0x8583);
    return (int16_t)raw;
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
    bool on = false;

    while (1) {
        led_state_t state = get_led_state();
        on = !on;

        uint8_t r = 0, g = 0, b = 0;
        if (on) {
            switch (state) {
            case LED_STATE_WIFI_DOWN:
                r = 32; g = 0; b = 0;    // 红
                break;
            case LED_STATE_WIFI_UP_NO_APP:
                r = 32; g = 16; b = 0;   // 黄
                break;
            case LED_STATE_APP_CONNECTED:
                r = 0; g = 32; b = 0;    // 绿
                break;
            default:
                break;
            }
        }

        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
        vTaskDelay(pdMS_TO_TICKS(250));
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
        .sin_port = htons(UDP_PORT),
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

static void task_net_rx(void *arg)
{
    (void)arg;
    char rx_buf[128];

    while (1) {
        struct sockaddr_in src_addr = {0};
        socklen_t addr_len = sizeof(src_addr);
        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&src_addr, &addr_len);
        if (len > 0) {
            rx_buf[len] = '\0';
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
        }
    }
}

static void task_heartbeat(void *arg)
{
    (void)arg;
    const char *msg = "{\"type\":\"esp_heartbeat\",\"dev\":\"" DEVICE_NAME "\"}";

    struct sockaddr_in bcast_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    while (1) {
        if (!client_is_connected()) {
            sendto(s_udp_sock, msg, strlen(msg), 0,
                   (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
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
    char tx_buf[256];

    while (1) {
        if (client_is_connected()) {
            int adc_raw = 0;
            if (adc_oneshot_read(s_adc_handle, BATT_ADC_CHANNEL, &adc_raw) == ESP_OK) {
                float v_adc = (adc_raw / 4095.0f) * 3.3f;
                float v_batt = v_adc * 2.0f;
                int16_t ads_raw = ads1118_read_raw();

                int n = snprintf(tx_buf, sizeof(tx_buf),
                                 "{\"type\":\"data\",\"seq\":%" PRIu32
                                 ",\"uptime_ms\":%" PRIi64
                                 ",\"batt_raw\":%d,\"batt_v\":%.3f,\"ads_raw\":%d}",
                                 seq++, now_ms(), adc_raw, v_batt, ads_raw);
                if (n > 0) {
                    struct sockaddr_in dst = {0};
                    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
                    dst = s_client_state.addr;
                    xSemaphoreGive(s_client_mutex);

                    sendto(s_udp_sock, tx_buf, (size_t)n, 0,
                           (struct sockaddr *)&dst, sizeof(dst));
                }

                if (++log_div >= 100) {
                    log_div = 0;
                    ESP_LOGI(TAG, "TX seq=%" PRIu32 " batt=%.3fV ads=%d", seq, v_batt, ads_raw);
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot: ADC(GPIO7), SPI(CS38/MISO37/SCLK36/MOSI35), WS2812(GPIO48), UDP=%d", UDP_PORT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    s_client_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((s_wifi_event_group && s_client_mutex) ? ESP_OK : ESP_FAIL);

    init_batt_adc();
    init_ads1118_spi();
    init_ws2812();
    init_wifi_sta();

    xTaskCreate(task_status_led, "status_led", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "Waiting WiFi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    init_udp_socket();

    xTaskCreate(task_net_rx, "net_rx", 4096, NULL, 6, NULL);
    xTaskCreate(task_heartbeat, "heartbeat", 3072, NULL, 4, NULL);
    xTaskCreate(task_sampling_tx, "sampling_tx", 4096, NULL, 5, NULL);
}
