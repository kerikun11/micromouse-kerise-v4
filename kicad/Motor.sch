EESchema Schematic File Version 4
LIBS:KERISE-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 15
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
L KERISE:DRV8835 U2
U 1 1 57CA1E0F
P 5500 3600
F 0 "U2" H 5050 3950 60  0000 L CNN
F 1 "DRV8835" H 5650 3950 60  0000 L CNN
F 2 "mouse:DRV8835" H 5100 3750 60  0001 C CNN
F 3 "" H 5100 3750 60  0000 C CNN
	1    5500 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 57CA1E46
P 4800 4200
F 0 "#PWR011" H 4800 3950 50  0001 C CNN
F 1 "GND" H 4800 4050 50  0000 C CNN
F 2 "" H 4800 4200 50  0000 C CNN
F 3 "" H 4800 4200 50  0000 C CNN
	1    4800 4200
	1    0    0    -1  
$EndComp
Text Label 6600 3400 0    60   ~ 0
MT_OUT_L1
Text Label 6600 3500 0    60   ~ 0
MT_OUT_L2
Text HLabel 4800 3400 0    60   Input ~ 0
MT_IN_L1
Text HLabel 4800 3500 0    60   Input ~ 0
MT_IN_L2
Text Notes 3800 2700 0    100  ~ 0
Motor
$Comp
L Device:C C3
U 1 1 57CCEFE0
P 4000 3650
F 0 "C3" H 4025 3750 50  0000 L CNN
F 1 "0.1u" H 4025 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201_NoSilk" H 4038 3500 50  0001 C CNN
F 3 "" H 4000 3650 50  0000 C CNN
	1    4000 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR014
U 1 1 57CD5BEE
P 5600 3100
F 0 "#PWR014" H 5600 2950 50  0001 C CNN
F 1 "+BATT" H 5600 3240 50  0000 C CNN
F 2 "" H 5600 3100 50  0000 C CNN
F 3 "" H 5600 3100 50  0000 C CNN
	1    5600 3100
	1    0    0    -1  
$EndComp
Text Label 6600 3600 0    60   ~ 0
MT_OUT_R1
Text Label 6600 3700 0    60   ~ 0
MT_OUT_R2
Text HLabel 4800 3600 0    60   Input ~ 0
MT_IN_R1
Text HLabel 4800 3700 0    60   Input ~ 0
MT_IN_R2
Text Label 7700 3400 2    60   ~ 0
MT_OUT_L1
Text Label 7700 3500 2    60   ~ 0
MT_OUT_L2
Text Label 7700 3900 2    60   ~ 0
MT_OUT_R1
Text Label 7700 4000 2    60   ~ 0
MT_OUT_R2
$Comp
L power:+3.3V #PWR012
U 1 1 589CC6A5
P 5300 3100
F 0 "#PWR012" H 5300 2950 50  0001 C CNN
F 1 "+3.3V" H 5300 3240 50  0000 C CNN
F 2 "" H 5300 3100 50  0000 C CNN
F 3 "" H 5300 3100 50  0000 C CNN
	1    5300 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 589CC6C6
P 4000 3100
F 0 "#PWR09" H 4000 2950 50  0001 C CNN
F 1 "+3.3V" H 4000 3240 50  0000 C CNN
F 2 "" H 4000 3100 50  0000 C CNN
F 3 "" H 4000 3100 50  0000 C CNN
	1    4000 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR07
U 1 1 594721DF
P 3700 3100
F 0 "#PWR07" H 3700 2950 50  0001 C CNN
F 1 "+BATT" H 3700 3240 50  0000 C CNN
F 2 "" H 3700 3100 50  0000 C CNN
F 3 "" H 3700 3100 50  0000 C CNN
	1    3700 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 594721FA
P 3700 3650
F 0 "C2" H 3725 3750 50  0000 L CNN
F 1 "0.1u" H 3725 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0201_NoSilk" H 3738 3500 50  0001 C CNN
F 3 "" H 3700 3650 50  0000 C CNN
	1    3700 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5B66FC72
P 6350 3400
F 0 "L1" V 6300 3400 50  0000 C CNN
F 1 "47u" V 6425 3400 50  0000 C CNN
F 2 "mouse:L_4018" H 6350 3400 50  0001 C CNN
F 3 "" H 6350 3400 50  0001 C CNN
	1    6350 3400
	0    1    1    0   
$EndComp
$Comp
L Device:L L2
U 1 1 5B66FD2C
P 6350 3600
F 0 "L2" V 6300 3600 50  0000 C CNN
F 1 "47u" V 6425 3600 50  0000 C CNN
F 2 "mouse:L_4018" H 6350 3600 50  0001 C CNN
F 3 "" H 6350 3600 50  0001 C CNN
	1    6350 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 3100 5300 3200
Wire Wire Line
	5600 3100 5600 3200
Wire Wire Line
	5400 4200 5400 4100
Wire Wire Line
	5700 4200 5700 4100
Wire Wire Line
	4800 4200 4800 3900
Wire Wire Line
	4800 3900 4900 3900
Wire Wire Line
	4000 3500 4000 3100
Wire Wire Line
	4000 3800 4000 4200
Wire Wire Line
	6200 3400 6100 3400
Wire Wire Line
	6100 3500 6600 3500
Wire Wire Line
	4800 3400 4900 3400
Wire Wire Line
	4900 3500 4800 3500
Wire Wire Line
	4800 3600 4900 3600
Wire Wire Line
	4900 3700 4800 3700
Wire Wire Line
	7700 4000 7800 4000
Wire Wire Line
	7800 3900 7700 3900
Wire Wire Line
	7700 3400 7800 3400
Wire Wire Line
	7800 3500 7700 3500
Wire Wire Line
	3700 4200 3700 3800
Wire Wire Line
	3700 3500 3700 3100
Wire Wire Line
	6600 3400 6500 3400
Text Label 6200 3400 1    60   ~ 0
MT_OUT_L_L
Text Label 6200 3600 3    60   ~ 0
MT_OUT_L_R
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5B6F504A
P 8000 3400
F 0 "J1" H 8079 3392 50  0000 L CNN
F 1 "Motor_L" H 8079 3301 50  0000 L CNN
F 2 "mouse:SMD_conn_2" H 8000 3400 50  0001 C CNN
F 3 "~" H 8000 3400 50  0001 C CNN
	1    8000 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5B6F50A6
P 8000 3900
F 0 "J2" H 8080 3892 50  0000 L CNN
F 1 "Motor_R" H 8080 3801 50  0000 L CNN
F 2 "mouse:SMD_conn_2" H 8000 3900 50  0001 C CNN
F 3 "~" H 8000 3900 50  0001 C CNN
	1    8000 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5B6FD311
P 5700 4200
F 0 "#PWR015" H 5700 3950 50  0001 C CNN
F 1 "GND" H 5700 4050 50  0000 C CNN
F 2 "" H 5700 4200 50  0000 C CNN
F 3 "" H 5700 4200 50  0000 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5B6FD364
P 4000 4200
F 0 "#PWR010" H 4000 3950 50  0001 C CNN
F 1 "GND" H 4000 4050 50  0000 C CNN
F 2 "" H 4000 4200 50  0000 C CNN
F 3 "" H 4000 4200 50  0000 C CNN
	1    4000 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5B7097B0
P 3700 4200
F 0 "#PWR08" H 3700 3950 50  0001 C CNN
F 1 "GND" H 3700 4050 50  0000 C CNN
F 2 "" H 3700 4200 50  0000 C CNN
F 3 "" H 3700 4200 50  0000 C CNN
	1    3700 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5B7097C9
P 5400 4200
F 0 "#PWR013" H 5400 3950 50  0001 C CNN
F 1 "GND" H 5400 4050 50  0000 C CNN
F 2 "" H 5400 4200 50  0000 C CNN
F 3 "" H 5400 4200 50  0000 C CNN
	1    5400 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3600 6100 3600
Wire Wire Line
	6100 3700 6600 3700
Wire Wire Line
	6500 3600 6600 3600
$EndSCHEMATC
