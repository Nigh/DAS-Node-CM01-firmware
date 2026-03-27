# DAS节点-电流采集-ESP32-Wifi

## 硬件连接
- ESP32-S3
	- BATT_ADC: IO7 (锂电 100k/100k分压)
	- SPI(ADS1118IDGS): CS(IO38) MISO(IO37) SCLK(IO36) MOSI(IO35)
	- WS2812: IO48
- ADS1118IDGS
	- AIN0: 1M/39k 分压
	- AIN1: 1.2v 参考
	- AIN2: ACS725-50AB
	- AIN3: ACS725-50AB

`AIN0` 和 `AIN1` 构成一个差分测量对。测量电压。  
`AIN2` 和 `AIN3` 测量一路电流，通过两片分流，最大测量到 100A

## WiFi 凭据外置

本项目已支持通过 Kconfig 注入 WiFi 信息，代码不再硬编码 SSID/密码。

### 1) 创建本地 secrets 文件

在 `das_node_cm01/` 下创建：`sdkconfig.secrets`

可直接从示例复制：

```bash
cd das_node_cm01
cp sdkconfig.secrets.example \
   sdkconfig.secrets
```

然后修改内容：

```ini
CONFIG_WIFI_SSID="你的WiFi名称"
CONFIG_WIFI_PASS="你的WiFi密码"
```

### 2) 构建时加载 secrets

`makefile` 已内置默认行为，在 `das_node_cm01/` 目录直接执行：

```bash
make
```
即可正常编译含Wifi信息的固件。


若需临时覆盖（比如切换到其他 secrets 文件）：

```bash
make SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dev.secrets" default
```
