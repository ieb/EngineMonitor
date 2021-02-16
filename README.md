# EngineMonitor

An engine monitor based on signalk/SenseESP using a ESP-WROOM-32 board from ebay somple pcb. It senses 1wire temperature, i2c pressure and temperature (BMP280), and has 2 digial inputs and 4 analogue inputs for monitoring a typical marine diesel engine. In addition there is a JDY-40 serial RF interface for polling remote sensors.

There are 2 modes of operation. One uses SignalK/SenserESP to send data over Wifi with configuration over Wifi. The other emits NMEA2000 messages to a NMEA200 bus, with configuration over Classic Bluetooth serial.

Most of the hard work is done in signalk/SenseESP, this project simply customises that work for this sensor.


The data is sensed and sent to a SignalK server which is discovered via mDSN over wifi. Wifi is
configured with a captive html ui when the ESP32 starts. Sensor settings my be re-configured over http and wifi switches. Use mDSN to discover the IP the sensor uses.

The PCB handles power and signal cleaning. It is single sided and can easilly be created using basic laser printer+CuCl etching at home, or using a CNC PCB Milling machine (FlatCAM+Candle).

Board used is a ESP-WROOM-32D based board with Wifi and Bluetooth, although BT is not used. Available from eBay. https://www.ebay.co.uk/itm/143297558340 30 pin DIL.
Datasheet of the chip https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf

Most of the remainder of this readme is notes.

# ESP32 pins

Not all ESP32 pins can be used, but most ESP32 pins can be remapped unlike Arduino boards. 

See PCB Schematic for details of how pins are used.
Pin numbers are the WROOM32 board pin numnbers, names are the pin name of the ESP32 chip.

Digital pulse in 1 -> Pin10/GPIO27
Digital pulse in 2 -> Pin9/GPIO26
RF Chip Enable (low to enable) -> Pin20/D4
RF Serial RX -> PIN21/RX2/GPIO16
RF Serail TX -> Pin22/TX2/GPIO17
RF Serial Enable -> Pin23/D5
CAN RX -> Pin24/D18
CAN TX -> Pin25/D19
1 Wire -> Pin26/D21
i2c SCL -> Pin29/D22
i2c SDA -> Pin30/D23

On the i2c bus is a BMP280 (temp/pressure) and an ADS1115 (4ch 16bit precision ADC) with a pin header for extension.
The Digital pulse is sharpened with a 74HC14 trigger.
CAN Trancever is a MCP2562
RF Serial is a JDY-40 with a Beken BK2461 controller and custom firmware. (2.4 GHz)

ESP32 ADC is non linear and frustrating to use, hence the i2c ACD.

D2 cant be used as its attached to the LED on the board.

GND on 14,17, VIN 5V on 15, 3.3V out on 16 
Other boards may vary but this seems to be a common pattern probably based on a standard Espresif design.

See signalk/SenseESP for details on how to develop.

![ESP-WROOM-32D Dev Board](ESPBoard.png)


# Connections

The board is intended to monitor the engine, other boards may monitor over the RF link, which avoids having to have many
devices connected to the N2K bus and long wires pulled through tight conduits.

## RF

### Battery Monitoring

Pro Mini with RF board using ACS758xCB at suitable range (eg ACS758ECB-200B-PFF-T, 10mV/A output bidirectional) 1 per battery bank sending data over RF on request, include a 1 wire sensor to sense terminal temperature. Should work as a single board screwed onto the battery terminals.

## 1 Wire

* Alternator Temp 
* Exhaust Temperature 
* Engine Room Temperature

Onewire sensors tend to not be rugged enough to attach directly to a engine block, see below
for coolant temp

## i2c

* BMP280 
   Engine room pressure, less that atmospheric indicates air flow problems.
   Engine room temperature (perhaps, however see 1 wire above)
* ADS1115
   ADC0 = Coolant Temperature Sensor
   ADC1 = Alternator Voltage
   ADC2 = Oil Pressure
   ADC3 = Fuel Level

## Analogue


* Alternator W terminal (/digital-1, setup as frequency sensor, blue)

1x 2 way plug


# Power

2 way -> 12->5V DC/DC -> board Vin

# Related information

This project is targeting a D2-40 engine.

Volvo Penta D2-40 engines like many other Volvo engines have a Mechanical Diesel Interface (MDI) that provides a J1939 interface to the sensors. MDIs are limited in what they sense, and so are probable limited in what they report.

Based on the workshop manual and physical connections the sensors available to the MDI are:

* Supply Voltage
* Coolant Temp, thermistor.
* Oil Pressure Switch
* Flywheel Hall sensor
* Glow plugs (current sensor tbc)
* Starter relay
* Buttons on the control pannel.

Note, there is no ECU on D1,D2 engines as the base engine is a fully mechanical Perkins unit with a mechanical injection pump.

Larger Volvo Penta engines with a full ECU will have many more sensors. A great explanation of J1939 can be found at https://www.csselectronics.com/screen/page/simple-intro-j1939-explained/language/en which is presented on a CAN physical transport.  J1939 is the heavy truck protocol, 250K baud, 29bit PGN. At L7 OSI stack it is J1979 (ISO 15031-5), and at L2 it is CAN (ISO 15765-4). Some commodity ELM327 scanners may be able to read and decode the packets with the correct plugs (eg viecar + CarScanner app).

J1939 is not NMEA2000, as the message structure is different and hence to connect a D2-40 MDI CAN Bus to a NMEA2000 CAN Bus requires a converter. Given how limited the set of sensors is, and how simple the MDI is it may not be worth it for MDI based engines. Volvo Penta Gateways retail at about 300 GBP, the Digital 