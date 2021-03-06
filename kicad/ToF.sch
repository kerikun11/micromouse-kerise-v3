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
Sheet 10 14
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
L VL53L0X U9
U 1 1 592B9F2C
P 4800 3300
F 0 "U9" H 4800 3450 60  0000 C CNN
F 1 "VL53L0X" H 4800 3300 60  0000 C CNN
F 2 "mouse:VL53L0X" H 4800 3300 60  0001 C CNN
F 3 "" H 4800 3300 60  0000 C CNN
	1    4800 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3300 4200 3500
Wire Wire Line
	4200 3400 4300 3400
Wire Wire Line
	4200 3500 4300 3500
Connection ~ 4200 3400
Wire Wire Line
	4200 3700 4200 4200
Wire Wire Line
	4200 4100 4300 4100
Wire Wire Line
	4200 4000 4300 4000
Connection ~ 4200 4100
Wire Wire Line
	4200 3900 4300 3900
Connection ~ 4200 4000
Wire Wire Line
	4200 3800 4300 3800
Connection ~ 4200 3900
Wire Wire Line
	4200 3700 4300 3700
Connection ~ 4200 3800
Text HLabel 4300 2400 0    60   Input ~ 0
SDA
Text HLabel 4300 2500 0    60   Input ~ 0
SCL
Wire Wire Line
	5300 3700 5800 3700
Wire Wire Line
	5300 3800 6100 3800
Wire Wire Line
	5400 3400 5300 3400
$Comp
L C_Small C29
U 1 1 592BA26F
P 3900 3800
F 0 "C29" H 3910 3870 50  0000 L CNN
F 1 "0.1u" H 3910 3720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201" H 3900 3800 50  0001 C CNN
F 3 "" H 3900 3800 50  0000 C CNN
	1    3900 3800
	1    0    0    -1  
$EndComp
$Comp
L C_Small C28
U 1 1 592BA2BF
P 3600 3800
F 0 "C28" H 3610 3870 50  0000 L CNN
F 1 "4.7u" H 3610 3720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3600 3800 50  0001 C CNN
F 3 "" H 3600 3800 50  0000 C CNN
	1    3600 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3300 3600 3700
Wire Wire Line
	3900 3700 3900 3300
Wire Wire Line
	3900 3900 3900 4200
Wire Wire Line
	3600 4200 3600 3900
Text Label 3600 3300 1    60   ~ 0
s_3.3V
Text Label 3900 3300 1    60   ~ 0
s_3.3V
Text Label 4200 3300 1    60   ~ 0
s_3.3V
Text Label 3600 4200 3    60   ~ 0
s_GND
Text Label 3900 4200 3    60   ~ 0
s_GND
Text Label 4200 4200 3    60   ~ 0
s_GND
Text Label 5400 3300 1    60   ~ 0
s_3.3V
Wire Wire Line
	5400 3300 5400 3400
$Comp
L CONN_01X04 P11
U 1 1 592AB778
P 4600 2450
F 0 "P11" H 4600 2700 50  0000 C CNN
F 1 "ToF_Host" V 4700 2450 50  0000 C CNN
F 2 "mouse:STAND_PR_HOST" H 4600 2450 50  0001 C CNN
F 3 "" H 4600 2450 50  0000 C CNN
	1    4600 2450
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P12
U 1 1 592AB803
P 5000 2450
F 0 "P12" H 5000 2700 50  0000 C CNN
F 1 "ToF_Slave" V 5100 2450 50  0000 C CNN
F 2 "mouse:STAND_PR_SLAVE" H 5000 2450 50  0001 C CNN
F 3 "" H 5000 2450 50  0000 C CNN
	1    5000 2450
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR60
U 1 1 592ABB63
P 4300 2700
F 0 "#PWR60" H 4300 2450 50  0001 C CNN
F 1 "GND" H 4300 2550 50  0000 C CNN
F 2 "" H 4300 2700 50  0000 C CNN
F 3 "" H 4300 2700 50  0000 C CNN
	1    4300 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2700 4300 2600
Wire Wire Line
	4300 2600 4400 2600
$Comp
L +3.3V #PWR59
U 1 1 592ABB9F
P 4300 2200
F 0 "#PWR59" H 4300 2050 50  0001 C CNN
F 1 "+3.3V" H 4300 2340 50  0000 C CNN
F 2 "" H 4300 2200 50  0000 C CNN
F 3 "" H 4300 2200 50  0000 C CNN
	1    4300 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2200 4300 2300
Wire Wire Line
	4300 2300 4400 2300
Text Label 5300 2300 0    60   ~ 0
s_3.3V
Text Label 5300 2600 0    60   ~ 0
s_GND
Text Label 5300 2500 0    60   ~ 0
s_SCL
Text Label 5300 2400 0    60   ~ 0
s_SDA
Wire Wire Line
	5300 2400 5200 2400
Wire Wire Line
	5200 2300 5300 2300
Wire Wire Line
	5300 2500 5200 2500
Wire Wire Line
	5200 2600 5300 2600
Text Label 5400 3800 0    60   ~ 0
s_SCL
Text Label 5400 3700 0    60   ~ 0
s_SDA
NoConn ~ 5300 3500
Wire Wire Line
	4400 2400 4300 2400
Wire Wire Line
	4300 2500 4400 2500
Text Notes 3450 1950 0    100  ~ 0
ToF
$Comp
L PWR_FLAG #FLG9
U 1 1 592CE2A7
P 5300 2300
F 0 "#FLG9" H 5300 2395 50  0001 C CNN
F 1 "PWR_FLAG" H 5300 2480 50  0000 C CNN
F 2 "" H 5300 2300 50  0000 C CNN
F 3 "" H 5300 2300 50  0000 C CNN
	1    5300 2300
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG10
U 1 1 592CE368
P 5300 2600
F 0 "#FLG10" H 5300 2695 50  0001 C CNN
F 1 "PWR_FLAG" H 5300 2780 50  0000 C CNN
F 2 "" H 5300 2600 50  0000 C CNN
F 3 "" H 5300 2600 50  0000 C CNN
	1    5300 2600
	-1   0    0    1   
$EndComp
$Comp
L R_Small R23
U 1 1 5932174C
P 5800 3500
F 0 "R23" H 5830 3520 50  0000 L CNN
F 1 "4.7k" H 5830 3460 50  0000 L CNN
F 2 "Resistors_SMD:R_0201" H 5800 3500 50  0001 C CNN
F 3 "" H 5800 3500 50  0000 C CNN
	1    5800 3500
	1    0    0    -1  
$EndComp
$Comp
L R_Small R24
U 1 1 593217CA
P 6100 3500
F 0 "R24" H 6130 3520 50  0000 L CNN
F 1 "4.7k" H 6130 3460 50  0000 L CNN
F 2 "Resistors_SMD:R_0201" H 6100 3500 50  0001 C CNN
F 3 "" H 6100 3500 50  0000 C CNN
	1    6100 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3800 6100 3600
Wire Wire Line
	5800 3700 5800 3600
Text Label 5800 3300 1    60   ~ 0
s_3.3V
Wire Wire Line
	5800 3300 5800 3400
Text Label 6100 3300 1    60   ~ 0
s_3.3V
Wire Wire Line
	6100 3300 6100 3400
$EndSCHEMATC
