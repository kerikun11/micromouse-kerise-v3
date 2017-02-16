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
LIBS:KERISEv3
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 13
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4900 4000 2    60   Input ~ 0
SWCLK
Text HLabel 4900 4200 2    60   Input ~ 0
SWDIO
Text HLabel 4900 4300 2    60   Input ~ 0
~RST
$Comp
L GND #PWR063
U 1 1 57CDA951
P 4800 4400
F 0 "#PWR063" H 4800 4150 50  0001 C CNN
F 1 "GND" H 4800 4250 50  0000 C CNN
F 2 "" H 4800 4400 50  0000 C CNN
F 3 "" H 4800 4400 50  0000 C CNN
	1    4800 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4100 4700 4100
Wire Wire Line
	4700 4000 4900 4000
Wire Wire Line
	4900 4200 4700 4200
Wire Wire Line
	4700 4300 4900 4300
$Comp
L GND #PWR064
U 1 1 57CDAC76
P 5850 4400
F 0 "#PWR064" H 5850 4150 50  0001 C CNN
F 1 "GND" H 5850 4250 50  0000 C CNN
F 2 "" H 5850 4400 50  0000 C CNN
F 3 "" H 5850 4400 50  0000 C CNN
	1    5850 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4100 5850 4400
Text HLabel 5950 4200 2    60   Input ~ 0
UART_RX
Text HLabel 5950 4300 2    60   Input ~ 0
UART_TX
Wire Wire Line
	5750 4200 5950 4200
Wire Wire Line
	5850 4100 5750 4100
$Comp
L CONN_01X04 P17
U 1 1 57CDB029
P 5550 4150
F 0 "P17" H 5550 4400 50  0000 C CNN
F 1 "UART" V 5650 4150 50  0000 C CNN
F 2 "Connectors_JST:JST_SH_BM04B-SRSS-TB_04x1.00mm_Straight" H 5550 4150 50  0001 C CNN
F 3 "" H 5550 4150 50  0000 C CNN
	1    5550 4150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5950 4300 5750 4300
Wire Wire Line
	5850 3900 5850 4000
Wire Wire Line
	5850 4000 5750 4000
Text Notes 4100 3600 0    100  ~ 0
COM Port
$Comp
L CONN_01X04 P16
U 1 1 57D2587E
P 4500 4150
F 0 "P16" H 4500 4400 50  0000 C CNN
F 1 "SWD" V 4600 4150 50  0000 C CNN
F 2 "Connectors_JST:JST_SH_BM04B-SRSS-TB_04x1.00mm_Straight" H 4500 4150 50  0001 C CNN
F 3 "" H 4500 4150 50  0000 C CNN
	1    4500 4150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4800 4400 4800 4100
$Comp
L +BATT #PWR065
U 1 1 58A05547
P 5850 3900
F 0 "#PWR065" H 5850 3750 50  0001 C CNN
F 1 "+BATT" H 5850 4040 50  0000 C CNN
F 2 "" H 5850 3900 50  0000 C CNN
F 3 "" H 5850 3900 50  0000 C CNN
	1    5850 3900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
