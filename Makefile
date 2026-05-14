.PHONY: build

build: 
	cd components/wifi_web_manager && cat <(xxd -i WEB_FILES/index.html) <(xxd -i WEB_FILES/script.js) > include/WEB_FILES.h
	idf.py build

flash: build
	idf.py flash -p /dev/ttyUSB0 -b 921600
	picocom /dev/ttyUSB0 -b 115200

clean:
	idf.py clean
