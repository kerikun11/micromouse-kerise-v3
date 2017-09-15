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
LIBS:KERISE-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 14
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
L Q_NMOS_GSD Q4
U 1 1 58AB90D4
P 5050 4800
F 0 "Q4" H 5350 4850 50  0000 R CNN
F 1 "DMN3065LW-7" H 5800 4750 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 5250 4900 50  0001 C CNN
F 3 "" H 5050 4800 50  0000 C CNN
	1    5050 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR054
U 1 1 58AB90DB
P 5150 5300
F 0 "#PWR054" H 5150 5050 50  0001 C CNN
F 1 "GND" H 5150 5150 50  0000 C CNN
F 2 "" H 5150 5300 50  0000 C CNN
F 3 "" H 5150 5300 50  0000 C CNN
	1    5150 5300
	1    0    0    -1  
$EndComp
Text HLabel 4550 4800 0    60   Input ~ 0
Fan
Wire Wire Line
	4550 4800 4850 4800
Wire Wire Line
	5150 5000 5150 5300
$Comp
L R R21
U 1 1 58AB90F7
P 4750 5050
F 0 "R21" V 4830 5050 50  0000 C CNN
F 1 "10k" V 4750 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 4680 5050 50  0001 C CNN
F 3 "" H 4750 5050 50  0000 C CNN
	1    4750 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4800 4750 4900
Connection ~ 4750 4800
$Comp
L GND #PWR055
U 1 1 58AB9100
P 4750 5300
F 0 "#PWR055" H 4750 5050 50  0001 C CNN
F 1 "GND" H 4750 5150 50  0000 C CNN
F 2 "" H 4750 5300 50  0000 C CNN
F 3 "" H 4750 5300 50  0000 C CNN
	1    4750 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 5300 4750 5200
$Comp
L CONN_01X02 P17
U 1 1 58AB920C
P 5450 4350
F 0 "P17" H 5450 4500 50  0000 C CNN
F 1 "Fan" V 5550 4350 50  0000 C CNN
F 2 "mouse:SMD_conn_2" H 5450 4350 50  0001 C CNN
F 3 "" H 5450 4350 50  0000 C CNN
	1    5450 4350
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR056
U 1 1 58AB927D
P 5150 4100
F 0 "#PWR056" H 5150 3950 50  0001 C CNN
F 1 "+BATT" H 5150 4240 50  0000 C CNN
F 2 "" H 5150 4100 50  0000 C CNN
F 3 "" H 5150 4100 50  0000 C CNN
	1    5150 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4100 5150 4300
Wire Wire Line
	5150 4300 5250 4300
Wire Wire Line
	5250 4400 5150 4400
Wire Wire Line
	5150 4400 5150 4600
Text Notes 4400 3900 0    60   ~ 0
Fan
Text Label 5150 4550 0    60   ~ 0
fan_to_fet
$EndSCHEMATC
