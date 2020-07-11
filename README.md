# EngineMonitor

An engine monitor based on signalk/SenseESP using a ESP-WROOM-32 board from ebay somple pcb. It senses 1wire temperature, i2c pressure and temperature (BMP280), and has 6 digial inputs and 6 analogue inputs for monitoring a typical marine diesel engine. Most of the hard work is done in signalk/SenseESP, this project simply customises that work for this sensor.

The data is sensed and sent to a SignalK server which is discovered via mDSN over wifi. Wifi is
configured with a captive html ui when the ESP32 starts. Sensor settings my be re-configured over http and wifi switches. Use mDSN to discover the IP the sensor uses.

The PCB handles power and signal cleaning. Its intended for through hole hand construction. It is single sided and can easilly be created using basic laser printer+CuCl etching at home. Tracks are all 1m or greater.

Board used is a ESP-WROOM-32D based board with Wifi and Bluetooth, although BT is not used. Available from eBay. https://www.ebay.co.uk/itm/143297558340 30 pin DIL.
Datasheet of the chip https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf

The digital headers are numbered 1-6 mapping to GPIO PINS 25,26,27,11,12,13 board pins 8,9,10,11,12,13. However GPIO11,12,13 cant be used as they are connected to the JTAG and Flash. There might some other pins available, wired with Jump wires. TODO, fix the PCB. D4,5, RX2, TX2, D18,D19, D23 all look like replacements. D2 is attached to the blue LED which is used for Wifi.

The analog headers are marked A-F mapping to GPIO PINS 33,32,35,34,VN,VP board pins 7,6,5,4,3,2

I2C has SCL on GIO22/D22, board pin 22 and SDA on GPIO21/D21 boardpin 26
1Wire is on GPIO15/D15 board pin 18

GND on 14,17, VIN 5V on 15, 3.3V out on 16 
Other boards may vary but this seems to be a common pattern probably based on a standard Espresif design.

See signalk/SenseESP for details on how to develop.

(ESPBoard.png)

