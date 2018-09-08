EESchema Schematic File Version 4
LIBS:KERISE-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 14
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
L Device:Q_NMOS_GSD Q7
U 1 1 58AB90D4
P 5050 4800
F 0 "Q7" H 5350 4850 50  0000 R CNN
F 1 "DMN3065LW-7" H 5800 4750 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 5250 4900 50  0001 C CNN
F 3 "" H 5050 4800 50  0000 C CNN
	1    5050 4800
	1    0    0    -1  
$EndComp
Text HLabel 4550 4800 0    60   Input ~ 0
Fan
Wire Wire Line
	4550 4800 4750 4800
Wire Wire Line
	5150 5000 5150 5300
$Comp
L Device:R R10
U 1 1 58AB90F7
P 4750 5050
F 0 "R10" V 4830 5050 50  0000 C CNN
F 1 "10k" V 4750 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0201_NoSilk" V 4680 5050 50  0001 C CNN
F 3 "" H 4750 5050 50  0000 C CNN
	1    4750 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4800 4750 4900
Connection ~ 4750 4800
$Comp
L power:GND #PWR042
U 1 1 58AB9100
P 4750 5300
F 0 "#PWR042" H 4750 5050 50  0001 C CNN
F 1 "GND" H 4750 5150 50  0000 C CNN
F 2 "" H 4750 5300 50  0000 C CNN
F 3 "" H 4750 5300 50  0000 C CNN
	1    4750 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 5300 4750 5200
$Comp
L power:+BATT #PWR043
U 1 1 58AB927D
P 5150 4100
F 0 "#PWR043" H 5150 3950 50  0001 C CNN
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
Wire Wire Line
	4750 4800 4850 4800
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 5B6F4A9E
P 5450 4300
F 0 "J8" H 5530 4292 50  0000 L CNN
F 1 "Fan" H 5530 4201 50  0000 L CNN
F 2 "mouse:SMD_conn_2" H 5450 4300 50  0001 C CNN
F 3 "~" H 5450 4300 50  0001 C CNN
	1    5450 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5B709ACC
P 5150 5300
F 0 "#PWR044" H 5150 5050 50  0001 C CNN
F 1 "GND" H 5150 5150 50  0000 C CNN
F 2 "" H 5150 5300 50  0000 C CNN
F 3 "" H 5150 5300 50  0000 C CNN
	1    5150 5300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
