EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:NMEA2000Module-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP32Dev U2
U 1 1 5EFF735F
P 5500 3650
F 0 "U2" H 5400 3300 60  0000 C CNN
F 1 "ESP32Dev" H 5400 3300 60  0000 C CNN
F 2 "Housings_DIP:DIP-30_1_ELL" H 5400 3300 60  0001 C CNN
F 3 "" H 5400 3300 60  0001 C CNN
	1    5500 3650
	1    0    0    -1  
$EndComp
$Comp
L CONN_3 K1
U 1 1 5EFF7888
P 8850 2250
F 0 "K1" V 8800 2250 50  0000 C CNN
F 1 "1W1" V 8900 2250 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8850 2250 60  0001 C CNN
F 3 "" H 8850 2250 60  0001 C CNN
	1    8850 2250
	0    -1   -1   0   
$EndComp
$Comp
L CONN_4 P7
U 1 1 5EFF7A15
P 8900 3500
F 0 "P7" V 8850 3500 50  0000 C CNN
F 1 "I2C1" V 8950 3500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 8900 3500 60  0001 C CNN
F 3 "" H 8900 3500 60  0001 C CNN
	1    8900 3500
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR01
U 1 1 5EFF7B89
P 9100 3100
F 0 "#PWR01" H 9100 3100 30  0001 C CNN
F 1 "GND" H 9100 3030 30  0001 C CNN
F 2 "" H 9100 3100 60  0001 C CNN
F 3 "" H 9100 3100 60  0001 C CNN
	1    9100 3100
	1    0    0    -1  
$EndComp
Text GLabel 9150 2900 2    60   Input ~ 0
1Wire
$Comp
L GND #PWR02
U 1 1 5EFF7E93
P 9550 4050
F 0 "#PWR02" H 9550 4050 30  0001 C CNN
F 1 "GND" H 9550 3980 30  0001 C CNN
F 2 "" H 9550 4050 60  0001 C CNN
F 3 "" H 9550 4050 60  0001 C CNN
	1    9550 4050
	1    0    0    -1  
$EndComp
Text GLabel 9050 4150 2    60   Input ~ 0
SDA
Text GLabel 9050 4050 2    60   Input ~ 0
SCL
Text GLabel 7700 2950 2    60   Output ~ 0
1Wire
$Comp
L GND #PWR03
U 1 1 5EFF8255
P 7600 4800
F 0 "#PWR03" H 7600 4800 30  0001 C CNN
F 1 "GND" H 7600 4730 30  0001 C CNN
F 2 "" H 7600 4800 60  0001 C CNN
F 3 "" H 7600 4800 60  0001 C CNN
	1    7600 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5EFF83F2
P 3450 4850
F 0 "#PWR04" H 3450 4850 30  0001 C CNN
F 1 "GND" H 3450 4780 30  0001 C CNN
F 2 "" H 3450 4850 60  0001 C CNN
F 3 "" H 3450 4850 60  0001 C CNN
	1    3450 4850
	1    0    0    -1  
$EndComp
Text GLabel 7650 2350 2    60   Output ~ 0
SCL
Text GLabel 7650 2150 2    60   Output ~ 0
SDA
$Comp
L R R12
U 1 1 5EFF8634
P 9900 1450
F 0 "R12" V 9980 1450 40  0000 C CNN
F 1 "4K7" V 9907 1451 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9830 1450 30  0001 C CNN
F 3 "" H 9900 1450 30  0000 C CNN
	1    9900 1450
	1    0    0    -1  
$EndComp
Text GLabel 10200 1700 2    60   Input ~ 0
1Wire
$Comp
L R R8
U 1 1 5EFF8D09
P 9950 5200
F 0 "R8" V 10030 5200 40  0000 C CNN
F 1 "10K" V 9957 5201 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9880 5200 30  0001 C CNN
F 3 "" H 9950 5200 30  0000 C CNN
	1    9950 5200
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5EFF8E43
P 10400 5200
F 0 "R10" V 10480 5200 40  0000 C CNN
F 1 "10K" V 10407 5201 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10330 5200 30  0001 C CNN
F 3 "" H 10400 5200 30  0000 C CNN
	1    10400 5200
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 5EFF8E79
P 10900 5200
F 0 "R13" V 10980 5200 40  0000 C CNN
F 1 "10K" V 10907 5201 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10830 5200 30  0001 C CNN
F 3 "" H 10900 5200 30  0000 C CNN
	1    10900 5200
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5EFF96A2
P 9350 5150
F 0 "R6" V 9430 5150 40  0000 C CNN
F 1 "10K" V 9357 5151 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9280 5150 30  0001 C CNN
F 3 "" H 9350 5150 30  0000 C CNN
	1    9350 5150
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U1
U 1 1 5F006C3F
P 4350 7050
F 0 "U1" H 4500 7150 40  0000 C CNN
F 1 "74HC14" H 4550 6950 40  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_LongPads" H 4350 7050 60  0001 C CNN
F 3 "" H 4350 7050 60  0000 C CNN
	1    4350 7050
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U1
U 2 1 5F006E88
P 1950 7050
F 0 "U1" H 2100 7150 40  0000 C CNN
F 1 "74HC14" H 2150 6950 40  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_LongPads" H 1950 7050 60  0001 C CNN
F 3 "" H 1950 7050 60  0000 C CNN
	2    1950 7050
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5F0070AD
P 3600 6650
F 0 "R3" V 3680 6650 40  0000 C CNN
F 1 "10K" V 3600 6650 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3530 6650 30  0001 C CNN
F 3 "" H 3600 6650 30  0000 C CNN
	1    3600 6650
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5F00711F
P 1300 6700
F 0 "R1" V 1380 6700 40  0000 C CNN
F 1 "10K" V 1307 6701 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1230 6700 30  0001 C CNN
F 3 "" H 1300 6700 30  0000 C CNN
	1    1300 6700
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5F0071A0
P 1300 7350
F 0 "R2" V 1380 7350 40  0000 C CNN
F 1 "3K3" V 1307 7351 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1230 7350 30  0001 C CNN
F 3 "" H 1300 7350 30  0000 C CNN
	1    1300 7350
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5F00743C
P 3600 7350
F 0 "R4" V 3680 7350 40  0000 C CNN
F 1 "3K3" V 3607 7351 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3530 7350 30  0001 C CNN
F 3 "" H 3600 7350 30  0000 C CNN
	1    3600 7350
	-1   0    0    1   
$EndComp
$Comp
L C C2
U 1 1 5F00784A
P 1500 7400
F 0 "C2" H 1500 7500 40  0000 L CNN
F 1 "50nF" H 1506 7315 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1538 7250 30  0001 C CNN
F 3 "" H 1500 7400 60  0000 C CNN
	1    1500 7400
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5F007B30
P 3750 7400
F 0 "C4" H 3750 7500 40  0000 L CNN
F 1 "50nF" H 3756 7315 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3788 7250 30  0001 C CNN
F 3 "" H 3750 7400 60  0000 C CNN
	1    3750 7400
	1    0    0    -1  
$EndComp
Text GLabel 2600 7050 2    60   Output ~ 0
DIN1
Text GLabel 4950 7050 2    60   Output ~ 0
DIN2
Text GLabel 3350 3950 0    60   Input ~ 0
DIN1
Text GLabel 3350 3750 0    60   Input ~ 0
DIN2
Text GLabel 3450 5050 0    60   Input ~ 0
5v
$Comp
L GND #PWR05
U 1 1 5F00EA6F
P 6050 1600
F 0 "#PWR05" H 6050 1600 30  0001 C CNN
F 1 "GND" H 6050 1530 30  0001 C CNN
F 2 "" H 6050 1600 60  0001 C CNN
F 3 "" H 6050 1600 60  0001 C CNN
	1    6050 1600
	1    0    0    -1  
$EndComp
Text GLabel 6200 900  2    60   Output ~ 0
5v
$Comp
L GND #PWR06
U 1 1 5F012588
P 4300 7250
F 0 "#PWR06" H 4300 7250 30  0001 C CNN
F 1 "GND" H 4300 7180 30  0001 C CNN
F 2 "" H 4300 7250 60  0001 C CNN
F 3 "" H 4300 7250 60  0001 C CNN
	1    4300 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5F0142AC
P 1300 7700
F 0 "#PWR07" H 1300 7700 30  0001 C CNN
F 1 "GND" H 1300 7630 30  0001 C CNN
F 2 "" H 1300 7700 60  0001 C CNN
F 3 "" H 1300 7700 60  0001 C CNN
	1    1300 7700
	1    0    0    -1  
$EndComp
$Comp
L DIODE D1
U 1 1 5F0CBF72
P 4000 900
F 0 "D1" H 4000 1000 40  0000 C CNN
F 1 "DIODE" H 4000 800 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 4000 900 60  0001 C CNN
F 3 "" H 4000 900 60  0000 C CNN
	1    4000 900 
	1    0    0    -1  
$EndComp
$Comp
L MCP2562FD IC1
U 1 1 5F3F8B62
P 1700 1900
F 0 "IC1" H 1450 2200 40  0000 C CNN
F 1 "MCP2562FD" H 1950 1600 40  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 1700 1900 35  0000 C CIN
F 3 "" H 1700 1900 60  0000 C CNN
	1    1700 1900
	1    0    0    -1  
$EndComp
Text GLabel 1550 1200 0    60   Input ~ 0
5v
$Comp
L GND #PWR08
U 1 1 5F3F9A1A
P 1700 2500
F 0 "#PWR08" H 1700 2500 30  0001 C CNN
F 1 "GND" H 1700 2430 30  0001 C CNN
F 2 "" H 1700 2500 60  0001 C CNN
F 3 "" H 1700 2500 60  0001 C CNN
	1    1700 2500
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5F3FA3AC
P 2250 1300
F 0 "C3" H 2250 1400 40  0000 L CNN
F 1 "100nF" H 2256 1215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2288 1150 30  0001 C CNN
F 3 "" H 2250 1300 60  0000 C CNN
	1    2250 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5F3FA56B
P 2250 1650
F 0 "#PWR09" H 2250 1650 30  0001 C CNN
F 1 "GND" H 2250 1580 30  0001 C CNN
F 2 "" H 2250 1650 60  0001 C CNN
F 3 "" H 2250 1650 60  0001 C CNN
	1    2250 1650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5F3FA79D
P 800 2300
F 0 "C1" H 800 2400 40  0000 L CNN
F 1 "100nF" H 806 2215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 838 2150 30  0001 C CNN
F 3 "" H 800 2300 60  0000 C CNN
	1    800  2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5F3FA959
P 800 2650
F 0 "#PWR010" H 800 2650 30  0001 C CNN
F 1 "GND" H 800 2580 30  0001 C CNN
F 2 "" H 800 2650 60  0001 C CNN
F 3 "" H 800 2650 60  0001 C CNN
	1    800  2650
	1    0    0    -1  
$EndComp
Text GLabel 1050 1700 0    60   Output ~ 0
CAN-RX
Text GLabel 1050 1800 0    60   Input ~ 0
CAN-TX
Text Notes 2500 1350 0    60   ~ 0
CAN Trancever
$Comp
L GND #PWR011
U 1 1 5F3FD6B3
P 9850 6850
F 0 "#PWR011" H 9850 6850 30  0001 C CNN
F 1 "GND" H 9850 6780 30  0001 C CNN
F 2 "" H 9850 6850 60  0001 C CNN
F 3 "" H 9850 6850 60  0001 C CNN
	1    9850 6850
	1    0    0    -1  
$EndComp
Text GLabel 7150 5750 0    60   Input ~ 0
SCL
Text GLabel 7150 5850 0    60   Input ~ 0
SDA
$Comp
L GND #PWR012
U 1 1 5F3FE6D4
P 6550 5750
F 0 "#PWR012" H 6550 5750 30  0001 C CNN
F 1 "GND" H 6550 5680 30  0001 C CNN
F 2 "" H 6550 5750 60  0001 C CNN
F 3 "" H 6550 5750 60  0001 C CNN
	1    6550 5750
	1    0    0    -1  
$EndComp
$Comp
L CONN_4 P9
U 1 1 5F40035E
P 9800 4400
F 0 "P9" V 9750 4400 50  0000 C CNN
F 1 "ANALOG IN" V 9850 4400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 9800 4400 60  0001 C CNN
F 3 "" H 9800 4400 60  0001 C CNN
	1    9800 4400
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P1
U 1 1 5F403459
P 4200 6100
F 0 "P1" V 4150 6100 40  0000 C CNN
F 1 "PULSE_IN" V 4250 6100 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 4200 6100 60  0001 C CNN
F 3 "" H 4200 6100 60  0001 C CNN
	1    4200 6100
	1    0    0    -1  
$EndComp
Text Notes 1600 3900 0    60   ~ 0
RS-485\n
Text Notes 3750 6650 0    60   ~ 0
Digital In
Text GLabel 7700 3150 2    60   Output ~ 0
CAN-TX
Text GLabel 7700 3950 2    60   Input ~ 0
SERIALRX
Text GLabel 7700 3550 2    60   Output ~ 0
SERIALEN
Text GLabel 7700 3750 2    60   Output ~ 0
SERIALTX
Text GLabel 7700 3350 2    60   Input ~ 0
CAN-RX
$Comp
L MAX485/3485 IC2
U 1 1 5F408A98
P 1800 4750
F 0 "IC2" H 1550 5050 40  0000 C CNN
F 1 "MAX485/3485" H 2050 4450 40  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 1800 4750 35  0000 C CIN
F 3 "" H 1800 4750 60  0000 C CNN
	1    1800 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5F408DB6
P 1800 5250
F 0 "#PWR013" H 1800 5250 30  0001 C CNN
F 1 "GND" H 1800 5180 30  0001 C CNN
F 2 "" H 1800 5250 60  0001 C CNN
F 3 "" H 1800 5250 60  0001 C CNN
	1    1800 5250
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 5F408E33
P 2800 4750
F 0 "P2" V 2750 4750 40  0000 C CNN
F 1 "RS-485 A/B" V 2850 4750 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 2800 4750 60  0001 C CNN
F 3 "" H 2800 4750 60  0001 C CNN
	1    2800 4750
	1    0    0    -1  
$EndComp
Text GLabel 1100 4550 0    60   Input ~ 0
SERIALTX
Text GLabel 1100 4700 0    60   Input ~ 0
SERIALEN
Text GLabel 1100 4950 0    60   Output ~ 0
SERIALRX
$Comp
L R R5
U 1 1 5F409C50
P 2350 5250
F 0 "R5" V 2430 5250 40  0000 C CNN
F 1 "120R" V 2357 5251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2280 5250 30  0001 C CNN
F 3 "" H 2350 5250 30  0000 C CNN
	1    2350 5250
	1    0    0    -1  
$EndComp
NoConn ~ 7400 2550
NoConn ~ 7400 2750
NoConn ~ 3700 4350
NoConn ~ 3700 4550
Text Notes 3450 4600 0    60   ~ 0
JTAG
Text Notes 3450 4400 0    60   ~ 0
JTAG
Text Notes 7450 2800 0    60   ~ 0
USB
Text Notes 7450 2600 0    60   ~ 0
USB
NoConn ~ 3700 4150
Text Notes 3450 4200 0    60   ~ 0
UART
Text Notes 4400 2400 0    60   ~ 0
In only
NoConn ~ 7400 4550
NoConn ~ 7400 4350
NoConn ~ 3700 3550
NoConn ~ 3700 3350
NoConn ~ 3700 3150
NoConn ~ 3700 2950
NoConn ~ 3700 2750
NoConn ~ 3700 2550
NoConn ~ 3700 2350
NoConn ~ 3700 2150
$Comp
L JDY40 U3
U 1 1 5FEC3E25
P 1900 3350
F 0 "U3" H 1950 3200 60  0000 C CNN
F 1 "JDY40" H 1950 3350 60  0000 C CNN
F 2 "Divers:JDY-40-TH" H 1750 3300 60  0001 C CNN
F 3 "" H 1750 3300 60  0001 C CNN
	1    1900 3350
	1    0    0    -1  
$EndComp
NoConn ~ 1300 2900
NoConn ~ 1300 3000
NoConn ~ 1300 3100
NoConn ~ 1300 3200
NoConn ~ 1300 3300
NoConn ~ 1300 3400
NoConn ~ 1300 3500
NoConn ~ 1300 3600
Text GLabel 2700 3500 2    60   Input ~ 0
SERIALTX
Text GLabel 2700 3300 2    60   Input ~ 0
SERIALEN
Text GLabel 2700 3400 2    60   Output ~ 0
SERIALRX
Text GLabel 7800 4950 2    60   Output ~ 0
3v3
Text GLabel 7150 5550 0    60   Input ~ 0
3v3
Text GLabel 9050 4250 2    60   Input ~ 0
3v3
Text GLabel 9200 2800 2    60   Input ~ 0
3v3
Text GLabel 10150 1000 2    60   Input ~ 0
3v3
Text GLabel 2050 4200 2    60   Input ~ 0
3v3
Text GLabel 700  1950 0    60   Input ~ 0
3v3
Text GLabel 2650 3600 2    60   Input ~ 0
3v3
$Comp
L DCDCConverter U4
U 1 1 5FEC9BAE
P 5100 1200
F 0 "U4" H 5100 1200 60  0000 C CNN
F 1 "DCDCConverter" H 5100 1200 60  0000 C CNN
F 2 "Divers:DC-DC-Converter" H 5100 1200 60  0001 C CNN
F 3 "" H 5100 1200 60  0001 C CNN
	1    5100 1200
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 60180102
P 9350 6300
F 0 "R7" V 9430 6300 40  0000 C CNN
F 1 "2K2" V 9357 6301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9280 6300 30  0001 C CNN
F 3 "" H 9350 6300 30  0000 C CNN
	1    9350 6300
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 60180933
P 9100 6300
F 0 "C5" H 9100 6400 40  0000 L CNN
F 1 "10nF" H 9106 6215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9138 6150 30  0001 C CNN
F 3 "" H 9100 6300 60  0000 C CNN
	1    9100 6300
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 6018114E
P 9950 6300
F 0 "R9" V 10030 6300 40  0000 C CNN
F 1 "2K2" V 9957 6301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9880 6300 30  0001 C CNN
F 3 "" H 9950 6300 30  0000 C CNN
	1    9950 6300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 6018115A
P 9700 6300
F 0 "C6" H 9700 6400 40  0000 L CNN
F 1 "10nF" H 9706 6215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9738 6150 30  0001 C CNN
F 3 "" H 9700 6300 60  0000 C CNN
	1    9700 6300
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 601811F6
P 10400 6300
F 0 "R11" V 10480 6300 40  0000 C CNN
F 1 "2K2" V 10407 6301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10330 6300 30  0001 C CNN
F 3 "" H 10400 6300 30  0000 C CNN
	1    10400 6300
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 60181202
P 10150 6300
F 0 "C7" H 10150 6400 40  0000 L CNN
F 1 "10nF" H 10156 6215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10188 6150 30  0001 C CNN
F 3 "" H 10150 6300 60  0000 C CNN
	1    10150 6300
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 60181295
P 10900 6300
F 0 "R14" V 10980 6300 40  0000 C CNN
F 1 "2K2" V 10907 6301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10830 6300 30  0001 C CNN
F 3 "" H 10900 6300 30  0000 C CNN
	1    10900 6300
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 601812A1
P 10650 6300
F 0 "C8" H 10650 6400 40  0000 L CNN
F 1 "10nF" H 10656 6215 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10688 6150 30  0001 C CNN
F 3 "" H 10650 6300 60  0000 C CNN
	1    10650 6300
	1    0    0    -1  
$EndComp
Text GLabel 4400 6700 2    60   Input ~ 0
3v3
Text GLabel 7700 4150 2    60   Output ~ 0
RFEN
Text GLabel 2700 3200 2    60   Input ~ 0
RFEN
$Comp
L GND #PWR014
U 1 1 60188B7E
P 3150 3150
F 0 "#PWR014" H 3150 3150 30  0001 C CNN
F 1 "GND" H 3150 3080 30  0001 C CNN
F 2 "" H 3150 3150 60  0001 C CNN
F 3 "" H 3150 3150 60  0001 C CNN
	1    3150 3150
	1    0    0    -1  
$EndComp
Text GLabel 8700 1700 2    60   Input ~ 0
SDA
Text GLabel 9300 1700 2    60   Input ~ 0
SCL
$Comp
L R R16
U 1 1 6018B7EC
P 9100 1350
F 0 "R16" V 9180 1350 40  0000 C CNN
F 1 "4K7" V 9107 1351 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9030 1350 30  0001 C CNN
F 3 "" H 9100 1350 30  0000 C CNN
	1    9100 1350
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 6018B8A3
P 8400 1350
F 0 "R15" V 8480 1350 40  0000 C CNN
F 1 "4K7" V 8407 1351 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8330 1350 30  0001 C CNN
F 3 "" H 8400 1350 30  0000 C CNN
	1    8400 1350
	1    0    0    -1  
$EndComp
Text GLabel 9350 1000 2    60   Input ~ 0
3v3
Text GLabel 8650 1000 2    60   Input ~ 0
3v3
$Comp
L CONN_4 P4
U 1 1 60191F3D
P 10200 3200
F 0 "P4" V 10150 3200 50  0000 C CNN
F 1 "CAN" V 10250 3200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 10200 3200 60  0001 C CNN
F 3 "" H 10200 3200 60  0001 C CNN
	1    10200 3200
	0    1    -1   0   
$EndComp
Text GLabel 2550 1800 2    60   BiDi ~ 0
CANH
Text GLabel 2550 2000 2    60   BiDi ~ 0
CANL
$Comp
L R R17
U 1 1 601923F9
P 2300 2350
F 0 "R17" V 2380 2350 40  0000 C CNN
F 1 "120R" V 2307 2351 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2230 2350 30  0001 C CNN
F 3 "" H 2300 2350 30  0000 C CNN
	1    2300 2350
	1    0    0    -1  
$EndComp
Text GLabel 10550 3650 2    60   Output ~ 0
12v
$Comp
L GND #PWR015
U 1 1 601925D8
P 10850 3850
F 0 "#PWR015" H 10850 3850 30  0001 C CNN
F 1 "GND" H 10850 3780 30  0001 C CNN
F 2 "" H 10850 3850 60  0001 C CNN
F 3 "" H 10850 3850 60  0001 C CNN
	1    10850 3850
	1    0    0    -1  
$EndComp
Text GLabel 10500 4000 2    60   BiDi ~ 0
CANH
Text GLabel 10500 4100 2    60   BiDi ~ 0
CANL
Text GLabel 3650 900  0    60   Input ~ 0
12v
$Comp
L GND #PWR016
U 1 1 6019290E
P 3500 1600
F 0 "#PWR016" H 3500 1600 30  0001 C CNN
F 1 "GND" H 3500 1530 30  0001 C CNN
F 2 "" H 3500 1600 60  0001 C CNN
F 3 "" H 3500 1600 60  0001 C CNN
	1    3500 1600
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P3
U 1 1 60192BFD
P 3050 2550
F 0 "P3" V 3000 2550 40  0000 C CNN
F 1 "TERM_R" V 3100 2550 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3050 2550 60  0001 C CNN
F 3 "" H 3050 2550 60  0001 C CNN
	1    3050 2550
	1    0    0    -1  
$EndComp
$Comp
L BMP280 U5
U 1 1 60195C3D
P 10100 2400
F 0 "U5" H 10100 2400 60  0000 C CNN
F 1 "BMP280" H 10100 2500 60  0000 C CNN
F 2 "Divers:BMP280" H 10100 2400 60  0001 C CNN
F 3 "" H 10100 2400 60  0001 C CNN
	1    10100 2400
	1    0    0    -1  
$EndComp
Text GLabel 9750 2950 0    60   Input ~ 0
3v3
$Comp
L GND #PWR017
U 1 1 60195D59
P 9900 2950
F 0 "#PWR017" H 9900 2950 30  0001 C CNN
F 1 "GND" H 9900 2880 30  0001 C CNN
F 2 "" H 9900 2950 60  0001 C CNN
F 3 "" H 9900 2950 60  0001 C CNN
	1    9900 2950
	1    0    0    -1  
$EndComp
Text GLabel 10250 3000 2    60   Input ~ 0
SCL
Text GLabel 10250 2900 2    60   Input ~ 0
SDA
NoConn ~ 10200 2750
NoConn ~ 10300 2750
$Comp
L ADS1115 U6
U 1 1 60196FD0
P 7850 5800
F 0 "U6" H 7800 5950 60  0000 C CNN
F 1 "ADS1115" H 7800 6100 60  0000 C CNN
F 2 "Divers:ADS1115" H 7850 5800 60  0001 C CNN
F 3 "" H 7850 5800 60  0001 C CNN
	1    7850 5800
	0    1    1    0   
$EndComp
NoConn ~ 7550 5950
NoConn ~ 8600 5950
NoConn ~ 6650 4200
NoConn ~ 6750 3950
NoConn ~ 1650 1700
$Comp
L DIODE D3
U 1 1 6037572F
P 1000 7400
F 0 "D3" H 1000 7500 40  0000 C CNN
F 1 "DIODE" H 1000 7300 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 1000 7400 60  0001 C CNN
F 3 "" H 1000 7400 60  0000 C CNN
	1    1000 7400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 60375A8F
P 3600 7700
F 0 "#PWR018" H 3600 7700 30  0001 C CNN
F 1 "GND" H 3600 7630 30  0001 C CNN
F 2 "" H 3600 7700 60  0001 C CNN
F 3 "" H 3600 7700 60  0001 C CNN
	1    3600 7700
	1    0    0    -1  
$EndComp
$Comp
L DIODE D2
U 1 1 6037621C
P 1000 6750
F 0 "D2" H 1000 6850 40  0000 C CNN
F 1 "DIODE" H 1000 6650 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 1000 6750 60  0001 C CNN
F 3 "" H 1000 6750 60  0000 C CNN
	1    1000 6750
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D5
U 1 1 60376347
P 3450 7400
F 0 "D5" H 3450 7500 40  0000 C CNN
F 1 "DIODE" H 3450 7300 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 3450 7400 60  0001 C CNN
F 3 "" H 3450 7400 60  0000 C CNN
	1    3450 7400
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D4
U 1 1 60376665
P 3450 6800
F 0 "D4" H 3450 6900 40  0000 C CNN
F 1 "DIODE" H 3450 6700 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 3450 6800 60  0001 C CNN
F 3 "" H 3450 6800 60  0000 C CNN
	1    3450 6800
	0    -1   -1   0   
$EndComp
Text GLabel 3200 6550 0    60   Input ~ 0
3v3
Text GLabel 850  6500 0    60   Input ~ 0
3v3
Wire Wire Line
	8750 2600 8750 3000
Wire Wire Line
	8850 2600 8850 2900
Wire Wire Line
	8950 2600 8950 2800
Wire Wire Line
	9100 3000 9100 3100
Wire Wire Line
	7400 4950 7800 4950
Wire Wire Line
	7600 4800 7600 4750
Wire Wire Line
	7600 4750 7400 4750
Wire Wire Line
	3700 4750 3450 4750
Wire Wire Line
	3450 4750 3450 4850
Wire Wire Line
	7400 2350 7650 2350
Wire Wire Line
	10200 1700 9900 1700
Wire Wire Line
	3450 5050 3550 5050
Wire Wire Line
	3550 5050 3550 4950
Wire Wire Line
	3550 4950 3700 4950
Wire Wire Line
	5800 1500 6050 1500
Wire Wire Line
	6050 1500 6050 1600
Wire Wire Line
	4300 6700 4300 6950
Wire Wire Line
	4300 7150 4300 7250
Wire Wire Line
	2150 1800 2550 1800
Wire Wire Line
	2150 2000 2550 2000
Wire Wire Line
	1550 1200 1700 1200
Wire Wire Line
	1700 1050 1700 1500
Wire Wire Line
	1700 2300 1700 2500
Wire Wire Line
	700  1950 1250 1950
Wire Wire Line
	1250 2100 1000 2100
Wire Wire Line
	1000 2100 1000 2350
Wire Wire Line
	1000 2350 1700 2350
Connection ~ 1700 2350
Wire Wire Line
	1700 1050 2250 1050
Wire Wire Line
	2250 1050 2250 1100
Connection ~ 1700 1200
Wire Wire Line
	2250 1650 2250 1500
Wire Wire Line
	800  2100 800  1950
Connection ~ 800  1950
Wire Wire Line
	800  2650 800  2500
Wire Wire Line
	1050 1700 1250 1700
Wire Wire Line
	1050 1800 1250 1800
Wire Wire Line
	7650 2150 7400 2150
Wire Wire Line
	7400 3750 7700 3750
Wire Wire Line
	7400 3950 7700 3950
Wire Wire Line
	1100 4950 1350 4950
Wire Wire Line
	1100 4700 1200 4700
Wire Wire Line
	1200 4650 1200 4800
Wire Wire Line
	1200 4800 1350 4800
Wire Wire Line
	1100 4550 1350 4550
Wire Wire Line
	1350 4650 1200 4650
Connection ~ 1200 4700
Wire Wire Line
	2250 4850 2450 4850
Wire Wire Line
	2250 4650 2450 4650
Wire Wire Line
	1800 5150 1800 5250
Wire Wire Line
	2350 5000 2350 4850
Connection ~ 2350 4850
Wire Wire Line
	2350 5500 2350 5650
Wire Wire Line
	2350 5650 2450 5650
Wire Wire Line
	2450 5650 2450 4650
Wire Wire Line
	7400 3550 7700 3550
Wire Wire Line
	7700 3350 7400 3350
Wire Wire Line
	7700 3150 7400 3150
Wire Wire Line
	7400 2950 7700 2950
Wire Wire Line
	3350 3950 3700 3950
Wire Wire Line
	3350 3750 3700 3750
Wire Wire Line
	8750 3850 8750 4250
Wire Wire Line
	8850 3850 8850 4150
Wire Wire Line
	8950 3850 8950 4050
Wire Wire Line
	9050 3900 9050 3850
Wire Wire Line
	2500 3500 2700 3500
Wire Wire Line
	2500 3400 2700 3400
Wire Wire Line
	2500 3300 2700 3300
Wire Wire Line
	2500 3100 3150 3100
Wire Wire Line
	2050 4200 1800 4200
Wire Wire Line
	1800 4200 1800 4350
Wire Wire Line
	10150 1000 9900 1000
Wire Wire Line
	9900 1000 9900 1200
Wire Wire Line
	2650 3600 2500 3600
Wire Wire Line
	3650 900  3800 900 
Wire Wire Line
	3500 1500 4350 1500
Wire Wire Line
	4200 900  4350 900 
Wire Wire Line
	5800 900  6200 900 
Wire Wire Line
	9950 4750 9950 4800
Wire Wire Line
	9950 4800 10900 4800
Wire Wire Line
	10900 4800 10900 4950
Wire Wire Line
	10400 4950 10400 4850
Wire Wire Line
	10400 4850 9850 4850
Wire Wire Line
	9850 4850 9850 4750
Wire Wire Line
	9950 4950 9950 4900
Wire Wire Line
	9950 4900 9750 4900
Wire Wire Line
	9750 4900 9750 4750
Wire Wire Line
	9350 4900 9350 4850
Wire Wire Line
	9350 4850 9650 4850
Wire Wire Line
	9650 4850 9650 4750
Wire Wire Line
	9100 6500 9100 6650
Wire Wire Line
	9100 6650 10900 6650
Wire Wire Line
	10900 6650 10900 6550
Wire Wire Line
	10650 6650 10650 6500
Connection ~ 10650 6650
Wire Wire Line
	10400 6550 10400 6650
Connection ~ 10400 6650
Wire Wire Line
	10150 6650 10150 6500
Connection ~ 10150 6650
Wire Wire Line
	9950 6550 9950 6650
Connection ~ 9950 6650
Wire Wire Line
	9700 6650 9700 6500
Connection ~ 9700 6650
Wire Wire Line
	9350 6550 9350 6650
Connection ~ 9350 6650
Wire Wire Line
	9850 6850 9850 6650
Connection ~ 9850 6650
Wire Wire Line
	4400 6700 4300 6700
Wire Wire Line
	7400 4150 7700 4150
Wire Wire Line
	2700 3200 2500 3200
Wire Wire Line
	3150 3100 3150 3150
Wire Wire Line
	8700 1700 8400 1700
Wire Wire Line
	8400 1700 8400 1600
Wire Wire Line
	8650 1000 8400 1000
Wire Wire Line
	8400 1000 8400 1100
Wire Wire Line
	9350 1000 9100 1000
Wire Wire Line
	9100 1000 9100 1100
Wire Wire Line
	9300 1700 9100 1700
Wire Wire Line
	9100 1700 9100 1600
Wire Wire Line
	9550 3900 9550 4050
Wire Wire Line
	8950 2800 9200 2800
Wire Wire Line
	8850 2900 9150 2900
Wire Wire Line
	8750 3000 9100 3000
Wire Wire Line
	3500 1600 3500 1500
Wire Wire Line
	2300 2100 2300 1800
Connection ~ 2300 1800
Wire Wire Line
	2300 2600 2300 2650
Wire Wire Line
	2300 2650 2700 2650
Wire Wire Line
	2700 2450 2400 2450
Wire Wire Line
	2400 2450 2400 2000
Connection ~ 2400 2000
Wire Wire Line
	10050 3550 10050 4100
Wire Wire Line
	10050 4100 10500 4100
Wire Wire Line
	10500 4000 10150 4000
Wire Wire Line
	10150 4000 10150 3550
Wire Wire Line
	10250 3550 10250 3800
Wire Wire Line
	10250 3800 10850 3800
Wire Wire Line
	10850 3800 10850 3850
Wire Wire Line
	10550 3650 10350 3650
Wire Wire Line
	10350 3650 10350 3550
Wire Wire Line
	9050 3900 9550 3900
Wire Wire Line
	8950 4050 9050 4050
Wire Wire Line
	8850 4150 9050 4150
Wire Wire Line
	8750 4250 9050 4250
Wire Wire Line
	9750 2950 9800 2950
Wire Wire Line
	9800 2950 9800 2750
Wire Wire Line
	9900 2950 9900 2750
Wire Wire Line
	10250 3000 10000 3000
Wire Wire Line
	10000 3000 10000 2750
Wire Wire Line
	10250 2900 10100 2900
Wire Wire Line
	10100 2900 10100 2750
Wire Wire Line
	7150 5550 7550 5550
Wire Wire Line
	7150 5750 7550 5750
Wire Wire Line
	7150 5850 7550 5850
Wire Wire Line
	7550 5650 6550 5650
Wire Wire Line
	6550 5650 6550 5750
Wire Wire Line
	8600 5550 10900 5550
Wire Wire Line
	10900 5450 10900 6050
Wire Wire Line
	8600 5650 10400 5650
Wire Wire Line
	10400 5450 10400 6050
Wire Wire Line
	8600 5750 9950 5750
Wire Wire Line
	9950 5450 9950 6050
Wire Wire Line
	8600 5850 9350 5850
Wire Wire Line
	9350 5400 9350 6050
Connection ~ 9350 5850
Wire Wire Line
	9100 6100 9100 5850
Connection ~ 9100 5850
Wire Wire Line
	9700 6100 9700 5750
Connection ~ 9700 5750
Connection ~ 9950 5750
Wire Wire Line
	10150 6100 10150 5650
Connection ~ 10150 5650
Connection ~ 10400 5650
Wire Wire Line
	10650 6100 10650 5550
Connection ~ 10650 5550
Connection ~ 10900 5550
Wire Wire Line
	3450 7000 3450 7200
Connection ~ 3450 7050
Wire Wire Line
	3600 6900 3600 7100
Connection ~ 3600 7050
Wire Wire Line
	3750 7050 3750 7200
Connection ~ 3750 7050
Wire Wire Line
	1000 6950 1000 7200
Connection ~ 1000 7050
Wire Wire Line
	1300 6950 1300 7100
Connection ~ 1300 7050
Wire Wire Line
	1500 7050 1500 7200
Connection ~ 1500 7050
Wire Wire Line
	1500 7650 1500 7600
Wire Wire Line
	1000 7650 1500 7650
Wire Wire Line
	1000 7650 1000 7600
Wire Wire Line
	1300 7600 1300 7700
Connection ~ 1300 7650
Wire Wire Line
	850  6500 1000 6500
Wire Wire Line
	1000 6500 1000 6550
Wire Wire Line
	3450 6550 3450 6600
Wire Wire Line
	1000 7050 1500 7050
Wire Wire Line
	3200 6550 3450 6550
Wire Wire Line
	2400 7050 2600 7050
Wire Wire Line
	3450 7050 3900 7050
Wire Wire Line
	4800 7050 4950 7050
Wire Wire Line
	3750 7600 3750 7650
Wire Wire Line
	3750 7650 3450 7650
Wire Wire Line
	3450 7650 3450 7600
Wire Wire Line
	3600 7600 3600 7700
Connection ~ 3600 7650
Wire Wire Line
	3850 6200 3600 6200
Wire Wire Line
	3600 6200 3600 6400
Wire Wire Line
	3850 6000 1300 6000
Wire Wire Line
	1300 6000 1300 6450
$EndSCHEMATC
