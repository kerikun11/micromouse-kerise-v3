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
Sheet 5 13
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4700 2600 0    60   Input ~ 0
IR_RCV
Text Notes 3700 1900 0    100  ~ 0
Photo Reflector
$Comp
L CONN_01X04 P6
U 1 1 589BE652
P 5400 2550
AR Path="/57CF0B09/589BE652" Ref="P6"  Part="1" 
AR Path="/57CF2A30/589BE652" Ref="P12"  Part="1" 
F 0 "P12" H 5400 2800 50  0000 C CNN
F 1 "PR_Slave" V 5500 2550 50  0000 C CNN
F 2 "mouse:STAND_PR_SLAVE" H 5400 2550 50  0001 C CNN
F 3 "" H 5400 2550 50  0000 C CNN
	1    5400 2550
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q1
U 1 1 589BE75A
P 4300 4700
AR Path="/57CF0B09/589BE75A" Ref="Q1"  Part="1" 
AR Path="/57CF2A30/589BE75A" Ref="Q2"  Part="1" 
F 0 "Q2" H 4600 4750 50  0000 R CNN
F 1 "IRFML8244" H 4350 4900 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4500 4800 50  0001 C CNN
F 3 "" H 4300 4700 50  0000 C CNN
	1    4300 4700
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 589BE76D
P 4400 3550
AR Path="/57CF0B09/589BE76D" Ref="R6"  Part="1" 
AR Path="/57CF2A30/589BE76D" Ref="R11"  Part="1" 
F 0 "R11" V 4480 3550 50  0000 C CNN
F 1 "47" V 4400 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4330 3550 50  0001 C CNN
F 3 "" H 4400 3550 50  0000 C CNN
	1    4400 3550
	1    0    0    -1  
$EndComp
Text Label 4400 4450 0    60   ~ 0
LED_to_FET
Text Label 4400 5200 0    60   ~ 0
s_GND
Text Label 3800 4700 0    60   ~ 0
s_IR_LED
$Comp
L CONN_01X04 P5
U 1 1 589BE88F
P 5000 2550
AR Path="/57CF0B09/589BE88F" Ref="P5"  Part="1" 
AR Path="/57CF2A30/589BE88F" Ref="P11"  Part="1" 
F 0 "P11" H 5000 2800 50  0000 C CNN
F 1 "PR_Host" V 5100 2550 50  0000 C CNN
F 2 "mouse:STAND_PR_HOST" H 5000 2550 50  0001 C CNN
F 3 "" H 5000 2550 50  0000 C CNN
	1    5000 2550
	1    0    0    -1  
$EndComp
Text Label 5700 2500 0    60   ~ 0
s_IR_LED
Text Label 5700 2700 0    60   ~ 0
s_GND
Text Label 5700 2600 0    60   ~ 0
s_IR_RCV
Text Label 5700 2400 0    60   ~ 0
s_3.3V
$Comp
L R R7
U 1 1 589BEE3B
P 5600 4750
AR Path="/57CF0B09/589BEE3B" Ref="R7"  Part="1" 
AR Path="/57CF2A30/589BEE3B" Ref="R12"  Part="1" 
F 0 "R12" V 5680 4750 50  0000 C CNN
F 1 "100k" V 5600 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 5530 4750 50  0001 C CNN
F 3 "" H 5600 4750 50  0000 C CNN
	1    5600 4750
	1    0    0    -1  
$EndComp
Text Label 5600 5200 0    60   ~ 0
s_GND
Text HLabel 4700 2500 0    60   Input ~ 0
IR_LED
$Comp
L GND #PWR038
U 1 1 589BF265
P 4700 2800
AR Path="/57CF0B09/589BF265" Ref="#PWR038"  Part="1" 
AR Path="/57CF2A30/589BF265" Ref="#PWR042"  Part="1" 
F 0 "#PWR042" H 4700 2550 50  0001 C CNN
F 1 "GND" H 4700 2650 50  0000 C CNN
F 2 "" H 4700 2800 50  0000 C CNN
F 3 "" H 4700 2800 50  0000 C CNN
	1    4700 2800
	1    0    0    -1  
$EndComp
Text Label 4400 3300 0    60   ~ 0
s_3.3V
$Comp
L C C17
U 1 1 589EE263
P 4900 4750
AR Path="/57CF0B09/589EE263" Ref="C17"  Part="1" 
AR Path="/57CF2A30/589EE263" Ref="C21"  Part="1" 
F 0 "C21" H 4925 4850 50  0000 L CNN
F 1 "47u" H 4925 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4938 4600 50  0001 C CNN
F 3 "" H 4900 4750 50  0000 C CNN
	1    4900 4750
	1    0    0    -1  
$EndComp
Text Label 4900 5200 0    60   ~ 0
s_GND
Text Label 4500 3800 0    60   ~ 0
LED_to_C
$Comp
L +3.3V #PWR039
U 1 1 589CC209
P 4700 2300
AR Path="/57CF0B09/589CC209" Ref="#PWR039"  Part="1" 
AR Path="/57CF2A30/589CC209" Ref="#PWR043"  Part="1" 
F 0 "#PWR043" H 4700 2150 50  0001 C CNN
F 1 "+3.3V" H 4700 2440 50  0000 C CNN
F 2 "" H 4700 2300 50  0000 C CNN
F 3 "" H 4700 2300 50  0000 C CNN
	1    4700 2300
	1    0    0    -1  
$EndComp
Text Label 7300 4100 0    60   ~ 0
s_IR_RCV
Wire Wire Line
	4800 2500 4700 2500
Wire Wire Line
	4700 2400 4800 2400
Wire Wire Line
	4700 2700 4700 2800
Wire Wire Line
	4800 2700 4700 2700
Wire Wire Line
	4800 2600 4700 2600
Connection ~ 4400 3800
Wire Wire Line
	4900 3800 4400 3800
Wire Wire Line
	4900 4600 4900 3800
Wire Wire Line
	4900 5200 4900 4900
Wire Wire Line
	4700 2300 4700 2400
Wire Wire Line
	4400 3900 4400 3700
Wire Wire Line
	4400 4200 4400 4500
Wire Notes Line
	3700 5300 3700 3100
Wire Notes Line
	3700 5300 8300 5300
Wire Notes Line
	3700 3100 8300 3100
Wire Wire Line
	5600 4900 5600 5200
Wire Wire Line
	5600 3300 5600 3800
Connection ~ 5600 4200
Wire Wire Line
	5600 4100 5600 4600
Wire Wire Line
	5600 4200 5800 4200
Wire Wire Line
	5700 2700 5600 2700
Wire Wire Line
	5600 2600 5700 2600
Wire Wire Line
	5700 2500 5600 2500
Wire Wire Line
	5600 2400 5700 2400
Wire Wire Line
	4400 3400 4400 3300
Wire Wire Line
	4400 4900 4400 5200
Wire Wire Line
	3800 4700 4100 4700
Connection ~ 6200 4000
Wire Wire Line
	6200 3300 6200 3600
$Comp
L R R8
U 1 1 58A0483E
P 6200 3750
AR Path="/57CF0B09/58A0483E" Ref="R8"  Part="1" 
AR Path="/57CF2A30/58A0483E" Ref="R13"  Part="1" 
F 0 "R13" V 6280 3750 50  0000 C CNN
F 1 "1k" V 6200 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 6130 3750 50  0001 C CNN
F 3 "" H 6200 3750 50  0000 C CNN
	1    6200 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4100 7300 4100
Wire Wire Line
	7200 3500 7200 4100
Wire Wire Line
	7200 3700 7100 3700
Wire Wire Line
	7200 3500 7100 3500
Connection ~ 7200 3700
Wire Wire Line
	6800 3700 6400 3700
Wire Wire Line
	6400 3500 6400 4200
Connection ~ 6400 4200
Wire Wire Line
	6400 3500 6800 3500
Connection ~ 6400 3700
Wire Wire Line
	6700 3300 6700 3800
Wire Wire Line
	6200 4000 6500 4000
Connection ~ 7200 4100
Text Label 8400 3300 0    60   ~ 0
s_3.3V
$Comp
L C C19
U 1 1 57CF0F5E
P 6950 3500
AR Path="/57CF0B09/57CF0F5E" Ref="C19"  Part="1" 
AR Path="/57CF2A30/57CF0F5E" Ref="C23"  Part="1" 
F 0 "C23" H 6850 3600 50  0000 L CNN
F 1 "22p" H 6800 3400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201" H 6988 3350 50  0001 C CNN
F 3 "" H 6950 3500 50  0000 C CNN
	1    6950 3500
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 57CF0F57
P 6950 3700
AR Path="/57CF0B09/57CF0F57" Ref="R10"  Part="1" 
AR Path="/57CF2A30/57CF0F57" Ref="R15"  Part="1" 
F 0 "R15" V 7030 3700 50  0000 C CNN
F 1 "220k" V 6950 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 6880 3700 50  0001 C CNN
F 3 "" H 6950 3700 50  0000 C CNN
	1    6950 3700
	0    1    1    0   
$EndComp
Text Label 7900 5200 0    60   ~ 0
s_GND
$Comp
L C_Small C20
U 1 1 5925867D
P 7900 4100
AR Path="/57CF0B09/5925867D" Ref="C20"  Part="1" 
AR Path="/57CF2A30/5925867D" Ref="C24"  Part="1" 
F 0 "C24" H 7910 4170 50  0000 L CNN
F 1 "0.1u" H 7910 4020 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201" H 7900 4100 50  0001 C CNN
F 3 "" H 7900 4100 50  0000 C CNN
	1    7900 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5200 7900 4200
Text Label 6700 5200 0    60   ~ 0
s_GND
Wire Wire Line
	6700 4400 6700 5200
Text Label 6200 5200 0    60   ~ 0
s_GND
Wire Wire Line
	6200 4600 6200 5200
$Comp
L R R9
U 1 1 58A04889
P 6200 4450
AR Path="/57CF0B09/58A04889" Ref="R9"  Part="1" 
AR Path="/57CF2A30/58A04889" Ref="R14"  Part="1" 
F 0 "R14" V 6280 4450 50  0000 C CNN
F 1 "1k" V 6200 4450 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 6130 4450 50  0001 C CNN
F 3 "" H 6200 4450 50  0000 C CNN
	1    6200 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3900 6200 4300
Wire Wire Line
	6100 4200 6500 4200
$Comp
L C C18
U 1 1 57CF0F45
P 5950 4200
AR Path="/57CF0B09/57CF0F45" Ref="C18"  Part="1" 
AR Path="/57CF2A30/57CF0F45" Ref="C22"  Part="1" 
F 0 "C22" H 5975 4300 50  0000 L CNN
F 1 "0.01u" H 5975 4100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201" H 5988 4050 50  0001 C CNN
F 3 "" H 5950 4200 50  0000 C CNN
	1    5950 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 3300 8400 3300
Connection ~ 6700 3300
Connection ~ 6200 3300
Wire Wire Line
	7900 4000 7900 3300
Connection ~ 7900 3300
$Comp
L Partition P7
U 1 1 59497B18
P 6650 2450
AR Path="/57CF0B09/59497B18" Ref="P7"  Part="1" 
AR Path="/57CF2A30/59497B18" Ref="P13"  Part="1" 
F 0 "P13" H 6650 2550 60  0000 C CNN
F 1 "Partition" H 6650 2450 60  0000 C CNN
F 2 "mouse:Partition_Host" H 6650 2450 60  0001 C CNN
F 3 "" H 6650 2450 60  0001 C CNN
	1    6650 2450
	1    0    0    -1  
$EndComp
$Comp
L Partition P8
U 1 1 59497CA2
P 7050 2450
AR Path="/57CF0B09/59497CA2" Ref="P8"  Part="1" 
AR Path="/57CF2A30/59497CA2" Ref="P14"  Part="1" 
F 0 "P14" H 7050 2550 60  0000 C CNN
F 1 "Partition" H 7050 2450 60  0000 C CNN
F 2 "mouse:Partition_Slave" H 7050 2450 60  0001 C CNN
F 3 "" H 7050 2450 60  0001 C CNN
	1    7050 2450
	1    0    0    -1  
$EndComp
$Comp
L Partition P9
U 1 1 59497CF1
P 7450 2450
AR Path="/57CF0B09/59497CF1" Ref="P9"  Part="1" 
AR Path="/57CF2A30/59497CF1" Ref="P15"  Part="1" 
F 0 "P15" H 7450 2550 60  0000 C CNN
F 1 "Partition" H 7450 2450 60  0000 C CNN
F 2 "mouse:Partition_Host" H 7450 2450 60  0001 C CNN
F 3 "" H 7450 2450 60  0001 C CNN
	1    7450 2450
	1    0    0    -1  
$EndComp
$Comp
L Partition P10
U 1 1 59497D43
P 7850 2450
AR Path="/57CF0B09/59497D43" Ref="P10"  Part="1" 
AR Path="/57CF2A30/59497D43" Ref="P16"  Part="1" 
F 0 "P16" H 7850 2550 60  0000 C CNN
F 1 "Partition" H 7850 2450 60  0000 C CNN
F 2 "mouse:Partition_Slave" H 7850 2450 60  0001 C CNN
F 3 "" H 7850 2450 60  0001 C CNN
	1    7850 2450
	1    0    0    -1  
$EndComp
$Comp
L MCP6L91 U6
U 1 1 594AC13D
P 6800 4100
AR Path="/57CF0B09/594AC13D" Ref="U6"  Part="1" 
AR Path="/57CF2A30/594AC13D" Ref="U7"  Part="1" 
F 0 "U7" H 6800 4300 50  0000 L CNN
F 1 "AD8601" H 6800 3900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 6800 3700 50  0001 C CNN
F 3 "" H 6800 4300 50  0001 C CNN
	1    6800 4100
	1    0    0    -1  
$EndComp
$Comp
L D_Photo D3
U 1 1 594AC36C
P 5600 4000
AR Path="/57CF0B09/594AC36C" Ref="D3"  Part="1" 
AR Path="/57CF2A30/594AC36C" Ref="D5"  Part="1" 
F 0 "D5" H 5620 4070 50  0000 L CNN
F 1 "VEMD2020X01" H 5560 3890 50  0000 C CNN
F 2 "mouse:VISHAY_DIODE" H 5550 4000 50  0001 C CNN
F 3 "" H 5550 4000 50  0001 C CNN
	1    5600 4000
	0    1    1    0   
$EndComp
$Comp
L LED D2
U 1 1 594AC674
P 4400 4050
AR Path="/57CF0B09/594AC674" Ref="D2"  Part="1" 
AR Path="/57CF2A30/594AC674" Ref="D4"  Part="1" 
F 0 "D4" H 4400 4150 50  0000 C CNN
F 1 "VSMB294008G" H 4400 3950 50  0000 C CNN
F 2 "mouse:VISHAY_DIODE" H 4400 4050 50  0001 C CNN
F 3 "" H 4400 4050 50  0001 C CNN
	1    4400 4050
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG040
U 1 1 594B7401
P 8000 3600
AR Path="/57CF0B09/594B7401" Ref="#FLG040"  Part="1" 
AR Path="/57CF2A30/594B7401" Ref="#FLG044"  Part="1" 
F 0 "#FLG044" H 8000 3675 50  0001 C CNN
F 1 "PWR_FLAG" H 8000 3750 50  0000 C CNN
F 2 "" H 8000 3600 50  0001 C CNN
F 3 "" H 8000 3600 50  0001 C CNN
	1    8000 3600
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG041
U 1 1 594B744C
P 8000 4600
AR Path="/57CF0B09/594B744C" Ref="#FLG041"  Part="1" 
AR Path="/57CF2A30/594B744C" Ref="#FLG045"  Part="1" 
F 0 "#FLG045" H 8000 4675 50  0001 C CNN
F 1 "PWR_FLAG" H 8000 4750 50  0000 C CNN
F 2 "" H 8000 4600 50  0001 C CNN
F 3 "" H 8000 4600 50  0001 C CNN
	1    8000 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 4600 7900 4600
Connection ~ 7900 4600
Wire Wire Line
	7900 3600 8000 3600
Connection ~ 7900 3600
$EndSCHEMATC