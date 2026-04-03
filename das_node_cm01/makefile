.PHONY: default build flash monitor scan send ensure_wifi_config reconfigure clean fullclean

PORT ?= ttyACM0

SDKCONFIG_DEFAULTS_FALLBACK := $(if $(wildcard sdkconfig.defaults),sdkconfig.defaults;sdkconfig.secrets,sdkconfig.secrets)
SDKCONFIG_DEFAULTS ?= $(SDKCONFIG_DEFAULTS_FALLBACK)
export SDKCONFIG_DEFAULTS

ensure_wifi_config:
	@if [ -f sdkconfig ]; then \
		sed -i '/^CONFIG_WIFI_SSID=/d;/^CONFIG_WIFI_PASS=/d' sdkconfig; \
	fi
	@idf.py reconfigure

default: build

build: ensure_wifi_config
	idf.py build

reconfigure:
	idf.py reconfigure

flash:
	idf.py -p /dev/$(PORT) flash

monitor:
	idf.py -p /dev/$(PORT) monitor

clean:
	idf.py clean

fullclean:
	idf.py fullclean

# 以下两个保持在宿主机运行
scan:
	nc -ulvn -q 0 19000 < /dev/null

send:
	echo -n '{"type":"heartbeat"}' | nc -u -w1 192.168.50.122 19001
