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
Sheet 8 13
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
F 1 "IRFML8244" H 5700 4750 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5250 4900 50  0001 C CNN
F 3 "" H 5050 4800 50  0000 C CNN
	1    5050 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR050
U 1 1 58AB90DB
P 5150 5300
F 0 "#PWR050" H 5150 5050 50  0001 C CNN
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
L R R20
U 1 1 58AB90F7
P 4750 5050
F 0 "R20" V 4830 5050 50  0000 C CNN
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
L GND #PWR051
U 1 1 58AB9100
P 4750 5300
F 0 "#PWR051" H 4750 5050 50  0001 C CNN
F 1 "GND" H 4750 5150 50  0000 C CNN
F 2 "" H 4750 5300 50  0000 C CNN
F 3 "" H 4750 5300 50  0000 C CNN
	1    4750 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 5300 4750 5200
$Comp
L CONN_01X02 P18
U 1 1 58AB920C
P 5450 4350
F 0 "P18" H 5450 4500 50  0000 C CNN
F 1 "Fan" V 5550 4350 50  0000 C CNN
F 2 "mouse:SMD_conn_2" H 5450 4350 50  0001 C CNN
F 3 "" H 5450 4350 50  0000 C CNN
	1    5450 4350
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR052
U 1 1 58AB927D
P 5150 4100
F 0 "#PWR052" H 5150 3950 50  0001 C CNN
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
