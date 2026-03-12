

.PHONY: default flash monitor scan send ensure_wifi_config

SDKCONFIG_DEFAULTS_FALLBACK := $(if $(wildcard sdkconfig.defaults),sdkconfig.defaults;sdkconfig.secrets,sdkconfig.secrets)
SDKCONFIG_DEFAULTS ?= $(SDKCONFIG_DEFAULTS_FALLBACK)
export SDKCONFIG_DEFAULTS

ensure_wifi_config:
	@if [ -f sdkconfig ]; then \
		sed -i '/^CONFIG_WIFI_SSID=/d;/^CONFIG_WIFI_PASS=/d' sdkconfig; \
	fi
	@idf.py reconfigure

default: ensure_wifi_config
	idf.py build

flash: ensure_wifi_config
	idf.py -p /dev/ttyACM0 flash

monitor: ensure_wifi_config
	idf.py -p /dev/ttyACM0 monitor

scan:
	nc -ulvn -q 0 9000 < /dev/null

send:
	echo -n "hello" | nc -u -w1 192.168.50.122 9000
