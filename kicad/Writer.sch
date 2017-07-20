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
LIBS:KERISE
LIBS:74xgxx
LIBS:ac-dc
LIBS:actel
LIBS:allegro
LIBS:Altera
LIBS:analog_devices
LIBS:battery_management
LIBS:bbd
LIBS:bosch
LIBS:brooktre
LIBS:cmos_ieee
LIBS:dc-dc
LIBS:diode
LIBS:elec-unifil
LIBS:ESD_Protection
LIBS:ftdi
LIBS:gennum
LIBS:graphic
LIBS:hc11
LIBS:ir
LIBS:Lattice
LIBS:leds
LIBS:logo
LIBS:maxim
LIBS:mechanical
LIBS:microchip_dspic33dsc
LIBS:microchip_pic10mcu
LIBS:microchip_pic12mcu
LIBS:microchip_pic16mcu
LIBS:microchip_pic18mcu
LIBS:microchip_pic24mcu
LIBS:microchip_pic32mcu
LIBS:modules
LIBS:motor_drivers
LIBS:motors
LIBS:msp430
LIBS:nordicsemi
LIBS:nxp
LIBS:nxp_armmcu
LIBS:onsemi
LIBS:Oscillators
LIBS:Power_Management
LIBS:powerint
LIBS:pspice
LIBS:references
LIBS:relays
LIBS:rfcom
LIBS:sensors
LIBS:silabs
LIBS:stm8
LIBS:stm32
LIBS:supertex
LIBS:switches
LIBS:transf
LIBS:triac_thyristor
LIBS:ttl_ieee
LIBS:video
LIBS:wiznet
LIBS:Worldsemi
LIBS:Xicor
LIBS:zetex
LIBS:Zilog
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 11 15
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
L FT234XD U11
U 1 1 592C50E3
P 6100 3700
F 0 "U11" H 6100 4250 60  0000 C CNN
F 1 "FT234XD" H 6100 4100 60  0000 C CNN
F 2 "mouse:DFN-12-1EP_3x3mm_Pitch0.45mm" H 6100 4100 60  0001 C CNN
F 3 "" H 6100 4100 60  0000 C CNN
	1    6100 3700
	1    0    0    -1  
$EndComp
Text Label 5000 2300 0    60   ~ 0
USB_D+
Text Label 5000 2400 0    60   ~ 0
USB_D-
Text Label 5700 2100 0    60   ~ 0
USB_5V
Text Label 4800 2800 0    60   ~ 0
USB_GND
Text Label 6700 2400 2    60   ~ 0
USB_GND
Text Label 6700 2300 2    60   ~ 0
BAT_OUT
$Comp
L C_Small C31
U 1 1 592C6027
P 4900 3800
F 0 "C31" H 4910 3870 50  0000 L CNN
F 1 "0.1u" H 4910 3720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4900 3800 50  0001 C CNN
F 3 "" H 4900 3800 50  0000 C CNN
	1    4900 3800
	1    0    0    -1  
$EndComp
$Comp
L C_Small C29
U 1 1 592C6080
P 4700 3800
F 0 "C29" H 4710 3870 50  0000 L CNN
F 1 "0.1u" H 4710 3720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4700 3800 50  0001 C CNN
F 3 "" H 4700 3800 50  0000 C CNN
	1    4700 3800
	1    0    0    -1  
$EndComp
Text Label 5400 4000 2    60   ~ 0
USB_D-
Text Label 5400 4100 2    60   ~ 0
USB_D+
Text Label 5400 3300 2    60   ~ 0
USB_5V
Text Label 4700 4200 3    60   ~ 0
USB_GND
Text Label 4900 4200 3    60   ~ 0
USB_GND
Text Label 6800 4200 3    60   ~ 0
USB_GND
NoConn ~ 6700 3600
NoConn ~ 6700 3700
NoConn ~ 6700 3800
Text Label 6800 3500 0    60   ~ 0
USB_RX
Text Label 6800 3400 0    60   ~ 0
USB_TX
Text Label 6700 2500 2    60   ~ 0
USB_TX
Text Label 6700 2600 2    60   ~ 0
USB_RX
$Comp
L MCP73831 U10
U 1 1 592C7F7C
P 5500 5000
F 0 "U10" H 5500 5350 60  0000 C CNN
F 1 "MCP73831" H 5500 5250 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 5500 5000 60  0001 C CNN
F 3 "" H 5500 5000 60  0000 C CNN
	1    5500 5000
	1    0    0    -1  
$EndComp
Text Label 6100 4900 0    60   ~ 0
BAT_OUT
$Comp
L R_Small R24
U 1 1 592C8650
P 5000 5200
F 0 "R24" H 5030 5220 50  0000 L CNN
F 1 "4.7k" H 5030 5160 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 5000 5200 50  0001 C CNN
F 3 "" H 5000 5200 50  0000 C CNN
	1    5000 5200
	1    0    0    -1  
$EndComp
Text Label 4600 4900 2    60   ~ 0
USB_5V
$Comp
L C_Small C32
U 1 1 592C8B05
P 6000 5200
F 0 "C32" H 6010 5270 50  0000 L CNN
F 1 "4.7u" H 6010 5120 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6000 5200 50  0001 C CNN
F 3 "" H 6000 5200 50  0000 C CNN
	1    6000 5200
	1    0    0    -1  
$EndComp
Text Label 6000 5400 0    60   ~ 0
USB_GND
$Comp
L C_Small C30
U 1 1 592C8F8A
P 4700 5100
F 0 "C30" H 4710 5170 50  0000 L CNN
F 1 "4.7u" H 4710 5020 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4700 5100 50  0001 C CNN
F 3 "" H 4700 5100 50  0000 C CNN
	1    4700 5100
	1    0    0    -1  
$EndComp
$Comp
L R_Small R25
U 1 1 592C9443
P 6300 5000
F 0 "R25" H 6330 5020 50  0000 L CNN
F 1 "10k" H 6330 4960 50  0000 L CNN
F 2 "Resistors_SMD:R_0402" H 6300 5000 50  0001 C CNN
F 3 "" H 6300 5000 50  0000 C CNN
	1    6300 5000
	0    1    1    0   
$EndComp
$Comp
L LED_Small D10
U 1 1 592C9513
P 6600 5000
F 0 "D10" H 6550 5125 50  0000 L CNN
F 1 "Status" H 6500 4900 50  0000 L CNN
F 2 "LEDs:LED_0402" V 6600 5000 50  0001 C CNN
F 3 "" V 6600 5000 50  0000 C CNN
	1    6600 5000
	1    0    0    -1  
$EndComp
Text Label 6800 5000 0    60   ~ 0
USB_5V
Wire Wire Line
	6700 2600 6800 2600
Wire Wire Line
	6700 2400 6800 2400
Wire Wire Line
	6700 2300 6800 2300
Wire Wire Line
	5500 3500 5400 3500
Wire Wire Line
	5400 3500 5400 3700
Wire Wire Line
	4900 3600 5500 3600
Wire Wire Line
	5400 3700 5500 3700
Connection ~ 5400 3600
Wire Wire Line
	5400 4100 5500 4100
Wire Wire Line
	5500 4000 5400 4000
Wire Wire Line
	4900 3700 4900 3600
Wire Wire Line
	4700 3700 4700 3400
Wire Wire Line
	4700 3400 5500 3400
Wire Wire Line
	5400 3300 5400 3400
Connection ~ 5400 3400
Wire Wire Line
	4900 4200 4900 3900
Wire Wire Line
	4700 3900 4700 4200
Wire Wire Line
	6800 4000 6800 4200
Wire Wire Line
	6800 4100 6700 4100
Wire Wire Line
	6800 4000 6700 4000
Connection ~ 6800 4100
Wire Wire Line
	6800 3500 6700 3500
Wire Wire Line
	6700 3400 6800 3400
Wire Wire Line
	6700 2500 6800 2500
Wire Wire Line
	5500 5400 5500 5300
Wire Wire Line
	5000 5400 5000 5300
Wire Wire Line
	5100 5000 5000 5000
Wire Wire Line
	5000 5000 5000 5100
Wire Wire Line
	4600 4900 5100 4900
Wire Wire Line
	4700 5000 4700 4900
Connection ~ 4700 4900
Wire Wire Line
	4700 5400 4700 5200
Connection ~ 5500 5400
Connection ~ 5000 5400
Wire Wire Line
	6500 5000 6400 5000
Wire Wire Line
	6800 5000 6700 5000
Wire Wire Line
	6000 4900 6000 5100
Connection ~ 6000 4900
Wire Wire Line
	5900 4900 6100 4900
Wire Wire Line
	6000 5400 6000 5300
Wire Wire Line
	4700 5400 6000 5400
Wire Wire Line
	6200 5000 5900 5000
$Comp
L PWR_FLAG #FLG065
U 1 1 592CEAEC
P 4700 2900
F 0 "#FLG065" H 4700 2995 50  0001 C CNN
F 1 "PWR_FLAG" H 4700 3080 50  0000 C CNN
F 2 "" H 4700 2900 50  0000 C CNN
F 3 "" H 4700 2900 50  0000 C CNN
	1    4700 2900
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG066
U 1 1 592F31F5
P 4700 3400
F 0 "#FLG066" H 4700 3495 50  0001 C CNN
F 1 "PWR_FLAG" H 4700 3580 50  0000 C CNN
F 2 "" H 4700 3400 50  0000 C CNN
F 3 "" H 4700 3400 50  0000 C CNN
	1    4700 3400
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG J2
U 1 1 5942A7E5
P 4700 2300
F 0 "J2" H 4500 2750 50  0000 L CNN
F 1 "USB_OTG" H 4500 2650 50  0000 L CNN
F 2 "mouse:ZX62R-B-5P" H 4850 2250 50  0001 C CNN
F 3 "" H 4850 2250 50  0001 C CNN
	1    4700 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 2100 5700 2100
Wire Wire Line
	5000 2100 5300 2100
Wire Wire Line
	4700 2700 4700 2900
Connection ~ 4700 2800
NoConn ~ 5000 2500
$Comp
L CONN_01X04 P22
U 1 1 592C5604
P 7000 2450
F 0 "P22" H 7000 2700 50  0000 C CNN
F 1 "COM" V 7100 2450 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x04_Pitch1.27mm" H 7000 2450 50  0001 C CNN
F 3 "" H 7000 2450 50  0000 C CNN
	1    7000 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2800 4800 2800
Wire Wire Line
	4600 2700 4600 2800
Text Label 5100 2100 1    60   ~ 0
USB_BUS
$Comp
L Polyfuse_Small F1
U 1 1 594AE1FD
P 5400 2100
F 0 "F1" V 5325 2100 50  0000 C CNN
F 1 "FEMTOSMDC035F-02" V 5475 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" H 5450 1900 50  0001 L CNN
F 3 "" H 5400 2100 50  0001 C CNN
	1    5400 2100
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG067
U 1 1 594B7C3A
P 5200 2000
F 0 "#FLG067" H 5200 2075 50  0001 C CNN
F 1 "PWR_FLAG" H 5200 2150 50  0000 C CNN
F 2 "" H 5200 2000 50  0001 C CNN
F 3 "" H 5200 2000 50  0001 C CNN
	1    5200 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2100 5200 2000
Connection ~ 5200 2100
$EndSCHEMATC
