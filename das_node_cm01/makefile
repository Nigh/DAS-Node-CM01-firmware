

.PHONY: default flash


default:
	idf.py build

flash:
	idf.py -p /dev/ttyACM0 flash

monitor:
	idf.py -p /dev/ttyACM0 monitor

scan:
	nc -ulvn -q 0 9000 < /dev/null

send:
	echo -n "hello" | nc -u -w1 192.168.50.122 9000
