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


# Connections

Colors are wire colors on installed board

## 1 Wire

* Alternator Temp (electrical.alternators.12V.temperature, /oneWire-1)
* Charger Temp (propulsion.mainEngine.exhaustTemperature, /oneWire-2)
* Exhaust Temperature (electrical.chargers.12V.temperature, /oneWire-3)
* Fridge (electrical.fridge.main.temperature, /oneWire-4)

Onewire sensors tend to not be rugged enough to attach directly to a engine block, see below
for coolant temp

3 way plug, daisy chain as required. (vcc= orange, gnd=blue, 1wire=orange/white)

## i2c

* BMP280 

## Analogue

* A Engine Temperature Sender (/analog-1, setup for sender, blue/white)
* B Alternator output (/analog-2, setup for 10K/2.2K divider, orange/white)
* C Engine Battery (/analog-3, setup for 10K/2.2K divider, brown/white)
* D Service battery (/analog-4, setup for 10K/2.2K divider, brown)
* E spare (green)
* F Spare (green/white)

3x 2 way plugs

# Digital

* Alternator W terminal (/digital-1, setup as frequency sensor, blue)
* Spare (orange)

1x 2 way plug


# Power

2 way -> 12->5V DC/DC -> board Vin