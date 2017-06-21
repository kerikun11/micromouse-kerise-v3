EESchema Schematic File Version 2
LIBS:KERISEv3-rescue
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
Sheet 9 13
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
L GND #PWR048
U 1 1 57CDAC76
P 4800 4200
F 0 "#PWR048" H 4800 3950 50  0001 C CNN
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
Wire Wire Line
	4900 4100 4700 4100
Text Notes 4300 3450 0    100  ~ 0
COM Port
$Comp
L CONN_01X04 P17
U 1 1 592C4BBD
P 4500 3950
F 0 "P17" H 4500 4200 50  0000 C CNN
F 1 "COM" V 4600 3950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch1.27mm" H 4500 3950 50  0001 C CNN
F 3 "" H 4500 3950 50  0000 C CNN
	1    4500 3950
	-1   0    0    -1  
$EndComp
$Comp
L +BATT #PWR049
U 1 1 592C4BE4
P 4800 3700
F 0 "#PWR049" H 4800 3550 50  0001 C CNN
F 1 "+BATT" H 4800 3840 50  0000 C CNN
F 2 "" H 4800 3700 50  0000 C CNN
F 3 "" H 4800 3700 50  0000 C CNN
	1    4800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3700 4800 3800
Wire Wire Line
	4800 3800 4700 3800
$EndSCHEMATC
