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
LIBS:KERISEv3-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 14
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
L GND #PWR064
U 1 1 57CDAC76
P 4800 4200
F 0 "#PWR064" H 4800 3950 50  0001 C CNN
F 1 "GND" H 4800 4050 50  0000 C CNN
F 2 "" H 4800 4200 50  0000 C CNN
F 3 "" H 4800 4200 50  0000 C CNN
	1    4800 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3900 4800 4200
Text HLabel 4900 4000 2    60   Input ~ 0
UART_RX
Text HLabel 4900 4100 2    60   Input ~ 0
UART_TX
Wire Wire Line
	4700 4000 4900 4000
Wire Wire Line
	4800 3900 4700 3900
$Comp
L CONN_01X03 P16
U 1 1 57CDB029
P 4500 4000
F 0 "P16" H 4500 4250 50  0000 C CNN
F 1 "UART" V 4600 4000 50  0000 C CNN
F 2 "Connectors_JST:JST_SH_BM03B-SRSS-TB_03x1.00mm_Straight" H 4500 4000 50  0001 C CNN
F 3 "" H 4500 4000 50  0000 C CNN
	1    4500 4000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4900 4100 4700 4100
Text Notes 4100 3600 0    100  ~ 0
COM Port
$EndSCHEMATC
