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
Sheet 4 14
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
AR Path="/57CF2A30/589BE652" Ref="P8"  Part="1" 
AR Path="/597E1B35/589BE652" Ref="P17"  Part="1" 
AR Path="/597E1B39/589BE652" Ref="P19"  Part="1" 
F 0 "P19" H 5400 2800 50  0000 C CNN
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
AR Path="/597E1B35/589BE75A" Ref="Q5"  Part="1" 
AR Path="/597E1B39/589BE75A" Ref="Q6"  Part="1" 
F 0 "Q6" H 4600 4750 50  0000 R CNN
F 1 "DMN3065LW-7" H 4350 4900 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 4500 4800 50  0001 C CNN
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
AR Path="/597E1B35/589BE76D" Ref="R25"  Part="1" 
AR Path="/597E1B39/589BE76D" Ref="R30"  Part="1" 
F 0 "R30" V 4480 3550 50  0000 C CNN
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
AR Path="/57CF2A30/589BE88F" Ref="P7"  Part="1" 
AR Path="/597E1B35/589BE88F" Ref="P16"  Part="1" 
AR Path="/597E1B39/589BE88F" Ref="P18"  Part="1" 
F 0 "P18" H 5000 2800 50  0000 C CNN
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
AR Path="/597E1B35/589BEE3B" Ref="R26"  Part="1" 
AR Path="/597E1B39/589BEE3B" Ref="R31"  Part="1" 
F 0 "R31" V 5680 4750 50  0000 C CNN
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
L GND #PWR043
U 1 1 589BF265
P 4700 2800
AR Path="/57CF0B09/589BF265" Ref="#PWR043"  Part="1" 
AR Path="/57CF2A30/589BF265" Ref="#PWR047"  Part="1" 
AR Path="/597E1B35/589BF265" Ref="#PWR080"  Part="1" 
AR Path="/597E1B39/589BF265" Ref="#PWR084"  Part="1" 
F 0 "#PWR084" H 4700 2550 50  0001 C CNN
F 1 "GND" H 4700 2650 50  0000 C CNN
F 2 "" H 4700 2800 50  0000 C CNN
F 3 "" H 4700 2800 50  0000 C CNN
	1    4700 2800
	1    0    0    -1  
$EndComp
Text Label 4400 3300 0    60   ~ 0
s_3.3V
$Comp
L C C19
U 1 1 589EE263
P 4900 4750
AR Path="/57CF0B09/589EE263" Ref="C19"  Part="1" 
AR Path="/57CF2A30/589EE263" Ref="C23"  Part="1" 
AR Path="/597E1B35/589EE263" Ref="C33"  Part="1" 
AR Path="/597E1B39/589EE263" Ref="C37"  Part="1" 
F 0 "C37" H 4925 4850 50  0000 L CNN
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
AR Path="/597E1B35/58A0483E" Ref="R27"  Part="1" 
AR Path="/597E1B39/58A0483E" Ref="R32"  Part="1" 
F 0 "R32" V 6280 3750 50  0000 C CNN
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
L C C21
U 1 1 57CF0F5E
P 6950 3500
AR Path="/57CF0B09/57CF0F5E" Ref="C21"  Part="1" 
AR Path="/57CF2A30/57CF0F5E" Ref="C25"  Part="1" 
AR Path="/597E1B35/57CF0F5E" Ref="C35"  Part="1" 
AR Path="/597E1B39/57CF0F5E" Ref="C39"  Part="1" 
F 0 "C39" H 6850 3600 50  0000 L CNN
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
AR Path="/597E1B35/57CF0F57" Ref="R29"  Part="1" 
AR Path="/597E1B39/57CF0F57" Ref="R34"  Part="1" 
F 0 "R34" V 7030 3700 50  0000 C CNN
F 1 "220k" V 6950 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0201" V 6880 3700 50  0001 C CNN
F 3 "" H 6950 3700 50  0000 C CNN
	1    6950 3700
	0    1    1    0   
$EndComp
Text Label 7900 5200 0    60   ~ 0
s_GND
$Comp
L C_Small C22
U 1 1 5925867D
P 7900 4100
AR Path="/57CF0B09/5925867D" Ref="C22"  Part="1" 
AR Path="/57CF2A30/5925867D" Ref="C26"  Part="1" 
AR Path="/597E1B35/5925867D" Ref="C36"  Part="1" 
AR Path="/597E1B39/5925867D" Ref="C40"  Part="1" 
F 0 "C40" H 7910 4170 50  0000 L CNN
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
AR Path="/597E1B35/58A04889" Ref="R28"  Part="1" 
AR Path="/597E1B39/58A04889" Ref="R33"  Part="1" 
F 0 "R33" V 6280 4450 50  0000 C CNN
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
L C C20
U 1 1 57CF0F45
P 5950 4200
AR Path="/57CF0B09/57CF0F45" Ref="C20"  Part="1" 
AR Path="/57CF2A30/57CF0F45" Ref="C24"  Part="1" 
AR Path="/597E1B35/57CF0F45" Ref="C34"  Part="1" 
AR Path="/597E1B39/57CF0F45" Ref="C38"  Part="1" 
F 0 "C38" H 5975 4300 50  0000 L CNN
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
L MCP6L91 U6
U 1 1 594AC13D
P 6800 4100
AR Path="/57CF0B09/594AC13D" Ref="U6"  Part="1" 
AR Path="/57CF2A30/594AC13D" Ref="U7"  Part="1" 
AR Path="/597E1B35/594AC13D" Ref="U12"  Part="1" 
AR Path="/597E1B39/594AC13D" Ref="U13"  Part="1" 
F 0 "U13" H 6800 4300 50  0000 L CNN
F 1 "TLV316IDCKR" H 6800 3900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-353_SC-70-5" H 6800 3700 50  0001 C CNN
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
AR Path="/597E1B35/594AC36C" Ref="D11"  Part="1" 
AR Path="/597E1B39/594AC36C" Ref="D13"  Part="1" 
F 0 "D13" H 5620 4070 50  0000 L CNN
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
AR Path="/597E1B35/594AC674" Ref="D10"  Part="1" 
AR Path="/597E1B39/594AC674" Ref="D12"  Part="1" 
F 0 "D12" H 4400 4150 50  0000 C CNN
F 1 "VSMB294008G" H 4400 3950 50  0000 C CNN
F 2 "mouse:VISHAY_DIODE" H 4400 4050 50  0001 C CNN
F 3 "" H 4400 4050 50  0001 C CNN
	1    4400 4050
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG044
U 1 1 594B7401
P 8000 3600
AR Path="/57CF0B09/594B7401" Ref="#FLG044"  Part="1" 
AR Path="/57CF2A30/594B7401" Ref="#FLG048"  Part="1" 
AR Path="/597E1B35/594B7401" Ref="#FLG081"  Part="1" 
AR Path="/597E1B39/594B7401" Ref="#FLG085"  Part="1" 
F 0 "#FLG085" H 8000 3675 50  0001 C CNN
F 1 "PWR_FLAG" H 8000 3750 50  0000 C CNN
F 2 "" H 8000 3600 50  0001 C CNN
F 3 "" H 8000 3600 50  0001 C CNN
	1    8000 3600
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG045
U 1 1 594B744C
P 8000 4600
AR Path="/57CF0B09/594B744C" Ref="#FLG045"  Part="1" 
AR Path="/57CF2A30/594B744C" Ref="#FLG049"  Part="1" 
AR Path="/597E1B35/594B744C" Ref="#FLG082"  Part="1" 
AR Path="/597E1B39/594B744C" Ref="#FLG086"  Part="1" 
F 0 "#FLG086" H 8000 4675 50  0001 C CNN
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
$Comp
L VDDA #PWR046
U 1 1 59C54857
P 4700 2300
AR Path="/57CF0B09/59C54857" Ref="#PWR046"  Part="1" 
AR Path="/57CF2A30/59C54857" Ref="#PWR050"  Part="1" 
AR Path="/597E1B35/59C54857" Ref="#PWR083"  Part="1" 
AR Path="/597E1B39/59C54857" Ref="#PWR087"  Part="1" 
F 0 "#PWR087" H 4700 2150 50  0001 C CNN
F 1 "VDDA" H 4700 2450 50  0000 C CNN
F 2 "" H 4700 2300 50  0001 C CNN
F 3 "" H 4700 2300 50  0001 C CNN
	1    4700 2300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
