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
Sheet 9 14
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
L GND #PWR58
U 1 1 589E94E9
P 4100 3600
AR Path="/592ADBF2/589E94E9" Ref="#PWR58"  Part="1" 
AR Path="/59347BCD/589E94E9" Ref="#PWR62"  Part="1" 
F 0 "#PWR62" H 4100 3350 50  0001 C CNN
F 1 "GND" H 4100 3450 50  0000 C CNN
F 2 "" H 4100 3600 50  0000 C CNN
F 3 "" H 4100 3600 50  0000 C CNN
	1    4100 3600
	1    0    0    -1  
$EndComp
Text HLabel 4100 3100 0    60   Input ~ 0
CS
$Comp
L PWR_FLAG #FLG7
U 1 1 58CB39A3
P 5100 2900
AR Path="/592ADBF2/58CB39A3" Ref="#FLG7"  Part="1" 
AR Path="/59347BCD/58CB39A3" Ref="#FLG11"  Part="1" 
F 0 "#FLG11" H 5100 2995 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 3080 50  0000 C CNN
F 2 "" H 5100 2900 50  0000 C CNN
F 3 "" H 5100 2900 50  0000 C CNN
	1    5100 2900
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG8
U 1 1 58CB39EE
P 5100 3600
AR Path="/592ADBF2/58CB39EE" Ref="#FLG8"  Part="1" 
AR Path="/59347BCD/58CB39EE" Ref="#FLG12"  Part="1" 
F 0 "#FLG12" H 5100 3695 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 3780 50  0000 C CNN
F 2 "" H 5100 3600 50  0000 C CNN
F 3 "" H 5100 3600 50  0000 C CNN
	1    5100 3600
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR57
U 1 1 589CCACF
P 4100 2900
AR Path="/592ADBF2/589CCACF" Ref="#PWR57"  Part="1" 
AR Path="/59347BCD/589CCACF" Ref="#PWR61"  Part="1" 
F 0 "#PWR61" H 4100 2750 50  0001 C CNN
F 1 "+3.3V" H 4100 3040 50  0000 C CNN
F 2 "" H 4100 2900 50  0000 C CNN
F 3 "" H 4100 2900 50  0000 C CNN
	1    4100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3000 4100 3000
Wire Wire Line
	4100 3000 4100 2900
Wire Wire Line
	4100 3600 4100 3500
Wire Wire Line
	4100 3500 4200 3500
$Comp
L C_Small C27
U 1 1 592A96DD
P 5700 2900
AR Path="/592ADBF2/592A96DD" Ref="C27"  Part="1" 
AR Path="/59347BCD/592A96DD" Ref="C30"  Part="1" 
F 0 "C30" H 5710 2970 50  0000 L CNN
F 1 "0.1u" H 5710 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201" H 5700 2900 50  0001 C CNN
F 3 "" H 5700 2900 50  0000 C CNN
	1    5700 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3600 5100 3500
Wire Wire Line
	5100 3500 5000 3500
Wire Wire Line
	5100 2900 5100 3000
Wire Wire Line
	5100 3000 5000 3000
Text Label 5100 3000 0    60   ~ 0
s_VDD
Wire Wire Line
	5100 3300 5000 3300
Wire Wire Line
	5100 3200 5000 3200
Wire Wire Line
	5700 2500 5700 2800
Wire Wire Line
	5700 2700 6000 2700
Connection ~ 5700 2700
Wire Wire Line
	6000 2600 5700 2600
Connection ~ 5700 2600
Wire Wire Line
	5700 3000 5700 3700
Wire Wire Line
	5700 3600 6000 3600
Wire Wire Line
	5900 2900 5900 3600
Wire Wire Line
	5900 3300 6000 3300
Connection ~ 5900 3400
Wire Wire Line
	6000 3200 5900 3200
Connection ~ 5900 3300
Wire Wire Line
	5900 3100 6000 3100
Connection ~ 5900 3200
Wire Wire Line
	5900 3000 6000 3000
Connection ~ 5900 3100
Wire Wire Line
	5900 2900 6000 2900
Connection ~ 5900 3000
Wire Wire Line
	5900 3400 6000 3400
Connection ~ 5900 3600
Connection ~ 5700 3600
Wire Wire Line
	6900 2600 6800 2600
Wire Wire Line
	6900 2700 6800 2700
Wire Wire Line
	6900 2900 6800 2900
Wire Wire Line
	6800 2800 6900 2800
NoConn ~ 6800 3000
$Comp
L AS5048A U8
U 1 1 5933FF2E
P 6400 2900
AR Path="/592ADBF2/5933FF2E" Ref="U8"  Part="1" 
AR Path="/59347BCD/5933FF2E" Ref="U10"  Part="1" 
F 0 "U10" H 6400 3450 60  0000 C CNN
F 1 "AS5048A" H 6400 3300 60  0000 C CNN
F 2 "mouse:TSSOP-14_4.4x5mm_Pitch0.65mm" H 6400 3300 60  0001 C CNN
F 3 "" H 6400 3300 60  0000 C CNN
	1    6400 2900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P10
U 1 1 59340164
P 4800 3250
AR Path="/592ADBF2/59340164" Ref="P10"  Part="1" 
AR Path="/59347BCD/59340164" Ref="P14"  Part="1" 
F 0 "P14" H 4800 3600 50  0000 C CNN
F 1 "ENC_Slave" V 4900 3250 50  0000 C CNN
F 2 "mouse:STAND_AS_SLAVE" H 4800 3250 50  0001 C CNN
F 3 "" H 4800 3250 50  0000 C CNN
	1    4800 3250
	-1   0    0    -1  
$EndComp
Text Label 5100 3500 0    60   ~ 0
s_GND
Text Label 5700 2500 0    60   ~ 0
s_VDD
Text Label 5700 3700 0    60   ~ 0
s_GND
Text Label 6900 2600 0    60   ~ 0
s_CS
Text Label 6900 2700 0    60   ~ 0
s_SCLK
Text Label 6900 2800 0    60   ~ 0
s_MISO
Text Label 6900 2900 0    60   ~ 0
s_MOSI
Text Label 5100 3100 0    60   ~ 0
s_CS
Text Label 5100 3200 0    60   ~ 0
s_SCLK
Text Label 5100 3400 0    60   ~ 0
s_MISO
Text Label 5100 3300 0    60   ~ 0
s_MOSI
Wire Wire Line
	5100 3400 5000 3400
Wire Wire Line
	5000 3100 5100 3100
$Comp
L CONN_01X06 P9
U 1 1 59340673
P 4400 3250
AR Path="/592ADBF2/59340673" Ref="P9"  Part="1" 
AR Path="/59347BCD/59340673" Ref="P13"  Part="1" 
F 0 "P13" H 4400 3600 50  0000 C CNN
F 1 "ENC_Host" V 4500 3250 50  0000 C CNN
F 2 "mouse:STAND_AS_HOST" H 4400 3250 50  0001 C CNN
F 3 "" H 4400 3250 50  0000 C CNN
	1    4400 3250
	1    0    0    -1  
$EndComp
Text HLabel 4100 3200 0    60   Input ~ 0
SCLK
Text HLabel 4100 3400 0    60   Input ~ 0
MISO
Text HLabel 4100 3300 0    60   Input ~ 0
MOSI
Wire Wire Line
	4100 3400 4200 3400
Wire Wire Line
	4200 3300 4100 3300
Wire Wire Line
	4100 3200 4200 3200
Wire Wire Line
	4200 3100 4100 3100
$EndSCHEMATC
