object: i2c_master/i2c_master.cxx imudof/imudof.cxx Servo/Servo.cxx us/us.cxx stepper/stepper.cxx fdcr/fdcr.cxx wifi/wifi.cxx telescopeManager.cpp
	avr-gcc -mmcu=atmega1280 -std=c++11  -Ii2c_master -Ifdcr -Iimudof -IServo -Istepper -Ius -Iwifi -Os $^ -Wl,-u,vfscanf -lscanf_min -lm  -Wl,-u,vfprintf -lprintf_flt -lm -o manager.o
	avr-objcopy -j .text -j .data -O ihex manager.o manager.hex

send:
	avrdude -c arduino -p m1280 -P /dev/ttyUSB0 -b 57600 -F -U flash:w:manager.hex


wifi:   i2c_master/i2c_master.cxx imudof/imudof.cxx Servo/Servo.cxx us/us.cxx stepper/stepper.cxx fdcr/fdcr.cxx telescopeManager_wifiOnly.cxx
	avr-gcc -mmcu=atmega1280 -std=c++11  -Ii2c_master -Ifdcr -Iimudof -IServo -Istepper -Ius -Iwifi -Os $^ -Wl,-u,vfscanf -lscanf_min -lm  -Wl,-u,vfprintf -lprintf_flt -lm -o manager_wifi.o
	avr-objcopy -j .text -j .data -O ihex manager_wifi.o manager_wifi.hex
	avrdude -c arduino -p m1280 -P /dev/ttyUSB0 -b 57600 -F -U flash:w:manager_wifi.hex

serial: i2c_master/i2c_master.cxx imudof/imudof.cxx Servo/Servo.cxx us/us.cxx stepper/stepper.cxx fdcr/fdcr.cxx telescopeManager_serialOnly.cxx
	avr-gcc -mmcu=atmega1280 -std=c++11  -Ii2c_master -Ifdcr -Iimudof -IServo -Istepper -Ius -Iwifi -Os $^ -Wl,-u,vfscanf -lscanf_min -lm  -Wl,-u,vfprintf -lprintf_flt -lm -o manager_serial.o
	avr-objcopy -j .text -j .data -O ihex manager_serial.o manager_serial.hex
	avrdude -c arduino -p m1280 -P /dev/ttyUSB0 -b 57600 -F -U flash:w:manager_serial.hex

clean:
	rm *.o *.hex



