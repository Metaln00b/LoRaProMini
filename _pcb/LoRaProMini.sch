EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LoRaProMini"
Date "2021-07-05"
Rev "2.1"
Comp "(c) 2021 foorschtbar"
Comment1 "https://github.com/foorschtbar/LoRaProMini"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:RFM95W-868S2 U2
U 1 1 60600322
P 8950 2850
F 0 "U2" H 8950 2850 50  0000 C CNN
F 1 "RFM95W-868S2" H 9450 2250 50  0000 C CNN
F 2 "foorschtbar:HOPERF_RFM9XW_SMD" H 5650 4500 50  0001 C CNN
F 3 "https://www.hoperf.com/data/upload/portal/20181127/5bfcbea20e9ef.pdf" H 5650 4500 50  0001 C CNN
	1    8950 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:Antenna_Shield AE1
U 1 1 606080A4
P 9800 2250
F 0 "AE1" H 9944 2289 50  0000 L CNN
F 1 "Antenna_Shield" H 9944 2198 50  0000 L CNN
F 2 "foorschtbar:Antenna_SMA_uFL" H 9800 2350 50  0001 C CNN
F 3 "~" H 9800 2350 50  0001 C CNN
	1    9800 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 6060B262
P 9900 2600
F 0 "#PWR04" H 9900 2350 50  0001 C CNN
F 1 "GND" H 9905 2427 50  0000 C CNN
F 2 "" H 9900 2600 50  0001 C CNN
F 3 "" H 9900 2600 50  0001 C CNN
	1    9900 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 6060BCE9
P 8950 3600
F 0 "#PWR02" H 8950 3350 50  0001 C CNN
F 1 "GND" H 8955 3427 50  0000 C CNN
F 2 "" H 8950 3600 50  0001 C CNN
F 3 "" H 8950 3600 50  0001 C CNN
	1    8950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3450 8850 3550
Wire Wire Line
	8850 3550 8950 3550
Wire Wire Line
	9050 3550 9050 3450
Wire Wire Line
	8950 3450 8950 3550
Connection ~ 8950 3550
Wire Wire Line
	8950 3550 9050 3550
Wire Wire Line
	8950 3600 8950 3550
Wire Wire Line
	9900 2600 9900 2450
Wire Wire Line
	9800 2550 9800 2450
$Comp
L Device:C C1
U 1 1 6060DE4E
P 8500 2150
F 0 "C1" V 8248 2150 50  0000 C CNN
F 1 "10uF" V 8339 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8538 2000 50  0001 C CNN
F 3 "~" H 8500 2150 50  0001 C CNN
	1    8500 2150
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 6060EE1A
P 8950 1950
F 0 "#PWR01" H 8950 1800 50  0001 C CNN
F 1 "+3.3V" H 8965 2123 50  0000 C CNN
F 2 "" H 8950 1950 50  0001 C CNN
F 3 "" H 8950 1950 50  0001 C CNN
	1    8950 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6061042E
P 7900 2150
F 0 "#PWR03" H 7900 1900 50  0001 C CNN
F 1 "GND" H 7905 1977 50  0000 C CNN
F 2 "" H 7900 2150 50  0001 C CNN
F 3 "" H 7900 2150 50  0001 C CNN
	1    7900 2150
	1    0    0    -1  
$EndComp
Text GLabel 8450 2550 0    50   Input ~ 0
SPI_SCK
Text GLabel 8450 2650 0    50   Input ~ 0
SPI_MOSI
Text GLabel 8450 2750 0    50   Input ~ 0
SPI_MISO
Text GLabel 8450 2850 0    50   Input ~ 0
SPI_CS_LORA
Text GLabel 8450 3050 0    50   Input ~ 0
LORA_RST
NoConn ~ 9450 2750
NoConn ~ 9450 2850
NoConn ~ 9450 2950
Text GLabel 9450 3050 2    50   Input ~ 0
LORA_DIO2
Text GLabel 9450 3150 2    50   Input ~ 0
LORA_DIO1
Text GLabel 9450 3250 2    50   Input ~ 0
LORA_DIO0
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 U1
U 1 1 60612962
P 2250 1950
F 0 "U1" H 2250 2192 50  0000 C CNN
F 1 "MCP1700-3302E_SOT23" H 2250 2101 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 2250 2175 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 2250 1950 50  0001 C CNN
	1    2250 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 606082C8
P 1600 2100
F 0 "C2" H 1715 2146 50  0000 L CNN
F 1 "1uF" H 1715 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1638 1950 50  0001 C CNN
F 3 "~" H 1600 2100 50  0001 C CNN
	1    1600 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60608EBE
P 2850 2100
F 0 "C3" H 2965 2146 50  0000 L CNN
F 1 "1uF" H 2965 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2888 1950 50  0001 C CNN
F 3 "~" H 2850 2100 50  0001 C CNN
	1    2850 2100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 60609DBC
P 2450 5350
F 0 "J1" H 2550 5350 50  0000 L CNN
F 1 "Conn_01x02" H 2550 5250 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical" H 2450 5350 50  0001 C CNN
F 3 "~" H 2450 5350 50  0001 C CNN
	1    2450 5350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 6060ABC7
P 9600 5500
F 0 "J2" H 9680 5542 50  0000 L CNN
F 1 "Conn_01x03" H 9680 5451 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical" H 9600 5500 50  0001 C CNN
F 3 "~" H 9600 5500 50  0001 C CNN
	1    9600 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 6060C0CD
P 2550 6600
F 0 "J3" H 2630 6592 50  0000 L CNN
F 1 "Conn_01x04" H 2630 6501 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical" H 2550 6600 50  0001 C CNN
F 3 "~" H 2550 6600 50  0001 C CNN
	1    2550 6600
	1    0    0    -1  
$EndComp
Text Notes 2200 1550 0    50   ~ 0
LDO
Text Notes 9100 1600 0    50   ~ 0
LoRa
Text Notes 8850 4400 0    50   ~ 0
Aux 1-Wire
Text Notes 2000 6350 0    50   ~ 0
Aux I2C
$Comp
L power:GND #PWR0101
U 1 1 6060D611
P 1850 6600
F 0 "#PWR0101" H 1850 6350 50  0001 C CNN
F 1 "GND" V 1850 6400 50  0000 C CNN
F 2 "" H 1850 6600 50  0001 C CNN
F 3 "" H 1850 6600 50  0001 C CNN
	1    1850 6600
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 6060E38D
P 1850 6500
F 0 "#PWR0102" H 1850 6350 50  0001 C CNN
F 1 "+3.3V" V 1865 6628 50  0000 L CNN
F 2 "" H 1850 6500 50  0001 C CNN
F 3 "" H 1850 6500 50  0001 C CNN
	1    1850 6500
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 6060F51D
P 8450 5400
F 0 "#PWR0103" H 8450 5250 50  0001 C CNN
F 1 "+3.3V" V 8465 5528 50  0000 L CNN
F 2 "" H 8450 5400 50  0001 C CNN
F 3 "" H 8450 5400 50  0001 C CNN
	1    8450 5400
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 6061049C
P 3450 1950
F 0 "#PWR0104" H 3450 1800 50  0001 C CNN
F 1 "+3.3V" V 3450 2100 50  0000 L CNN
F 2 "" H 3450 1950 50  0001 C CNN
F 3 "" H 3450 1950 50  0001 C CNN
	1    3450 1950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60611209
P 2850 2250
F 0 "#PWR0105" H 2850 2000 50  0001 C CNN
F 1 "GND" H 2855 2077 50  0000 C CNN
F 2 "" H 2850 2250 50  0001 C CNN
F 3 "" H 2850 2250 50  0001 C CNN
	1    2850 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 606119BE
P 1600 2250
F 0 "#PWR0106" H 1600 2000 50  0001 C CNN
F 1 "GND" H 1605 2077 50  0000 C CNN
F 2 "" H 1600 2250 50  0001 C CNN
F 3 "" H 1600 2250 50  0001 C CNN
	1    1600 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1950 1600 1950
Wire Wire Line
	1600 1950 1400 1950
Connection ~ 1600 1950
Wire Wire Line
	2550 1950 2850 1950
Wire Wire Line
	2850 1950 3250 1950
Connection ~ 2850 1950
$Comp
L power:GND #PWR0107
U 1 1 60612A17
P 2250 2250
F 0 "#PWR0107" H 2250 2000 50  0001 C CNN
F 1 "GND" H 2255 2077 50  0000 C CNN
F 2 "" H 2250 2250 50  0001 C CNN
F 3 "" H 2250 2250 50  0001 C CNN
	1    2250 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 60612DF8
P 2000 5450
F 0 "#PWR0108" H 2000 5200 50  0001 C CNN
F 1 "GND" V 2000 5250 50  0000 C CNN
F 2 "" H 2000 5450 50  0001 C CNN
F 3 "" H 2000 5450 50  0001 C CNN
	1    2000 5450
	0    1    1    0   
$EndComp
Text Notes 1800 3250 0    50   ~ 0
Battery Measurement
$Comp
L Device:R R1
U 1 1 60956F0E
P 2050 3600
F 0 "R1" V 1843 3600 50  0000 C CNN
F 1 "10M" V 1934 3600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1980 3600 50  0001 C CNN
F 3 "~" H 2050 3600 50  0001 C CNN
	1    2050 3600
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 6095784B
P 2450 3600
F 0 "R2" V 2243 3600 50  0000 C CNN
F 1 "2.7M" V 2334 3600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2380 3600 50  0001 C CNN
F 3 "~" H 2450 3600 50  0001 C CNN
	1    2450 3600
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 60959353
P 2450 4100
F 0 "C4" V 2198 4100 50  0000 C CNN
F 1 "100nF" V 2289 4100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2488 3950 50  0001 C CNN
F 3 "~" H 2450 4100 50  0001 C CNN
	1    2450 4100
	0    1    1    0   
$EndComp
$Comp
L power:+BATT #PWR0109
U 1 1 6095A1AD
P 2000 5350
F 0 "#PWR0109" H 2000 5200 50  0001 C CNN
F 1 "+BATT" V 2000 5600 50  0000 C CNN
F 2 "" H 2000 5350 50  0001 C CNN
F 3 "" H 2000 5350 50  0001 C CNN
	1    2000 5350
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR0110
U 1 1 6095C230
P 1400 1950
F 0 "#PWR0110" H 1400 1800 50  0001 C CNN
F 1 "+BATT" V 1400 2100 50  0000 L CNN
F 2 "" H 1400 1950 50  0001 C CNN
F 3 "" H 1400 1950 50  0001 C CNN
	1    1400 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 5450 2250 5450
Wire Wire Line
	2250 5350 2000 5350
$Comp
L power:+BATT #PWR0111
U 1 1 60961A8F
P 1800 3600
F 0 "#PWR0111" H 1800 3450 50  0001 C CNN
F 1 "+BATT" V 1800 3750 50  0000 L CNN
F 2 "" H 1800 3600 50  0001 C CNN
F 3 "" H 1800 3600 50  0001 C CNN
	1    1800 3600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 609629D6
P 2600 4250
F 0 "#PWR0112" H 2600 4000 50  0001 C CNN
F 1 "GND" H 2605 4077 50  0000 C CNN
F 2 "" H 2600 4250 50  0001 C CNN
F 3 "" H 2600 4250 50  0001 C CNN
	1    2600 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 3600 1900 3600
Wire Wire Line
	2200 3600 2300 3600
Wire Wire Line
	2300 3600 2300 4100
Wire Wire Line
	2600 3600 2600 4100
Wire Wire Line
	2600 4100 2600 4250
Connection ~ 2600 4100
Text GLabel 5950 2400 2    50   Input ~ 0
MCU_A0
Text GLabel 2050 4100 0    50   Input ~ 0
MCU_A0
Wire Wire Line
	2050 4100 2300 4100
Connection ~ 2300 4100
Text GLabel 5950 2900 2    50   Input ~ 0
MCU_SCL
Text GLabel 5950 2800 2    50   Input ~ 0
MCU_SDA
Text GLabel 5950 2000 2    50   Input ~ 0
SPI_SCK
Text GLabel 5950 1900 2    50   Input ~ 0
SPI_MISO
Text GLabel 5950 1800 2    50   Input ~ 0
SPI_MOSI
Text GLabel 5950 1700 2    50   Input ~ 0
SPI_CS_LORA
Text GLabel 5950 1600 2    50   Input ~ 0
LORA_RST
Text GLabel 5950 3400 2    50   Input ~ 0
LORA_DIO2
Text GLabel 5950 3900 2    50   Input ~ 0
LORA_DIO1
Text GLabel 5950 1500 2    50   Input ~ 0
LORA_DIO0
$Comp
L Device:R R4
U 1 1 6097595C
P 2250 7050
F 0 "R4" H 2150 7000 50  0000 R CNN
F 1 "4,7K" H 2150 7100 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2180 7050 50  0001 C CNN
F 3 "~" H 2250 7050 50  0001 C CNN
	1    2250 7050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 6097622B
P 2100 7050
F 0 "R3" H 1900 7100 50  0000 L CNN
F 1 "4,7K" H 1850 7000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2030 7050 50  0001 C CNN
F 3 "~" H 2100 7050 50  0001 C CNN
	1    2100 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 609777C3
P 8900 5800
F 0 "R5" H 8830 5754 50  0000 R CNN
F 1 "4,7K" H 8830 5845 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8830 5800 50  0001 C CNN
F 3 "~" H 8900 5800 50  0001 C CNN
	1    8900 5800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 60977E11
P 8450 5600
F 0 "#PWR0116" H 8450 5350 50  0001 C CNN
F 1 "GND" V 8450 5400 50  0000 C CNN
F 2 "" H 8450 5600 50  0001 C CNN
F 3 "" H 8450 5600 50  0001 C CNN
	1    8450 5600
	0    1    1    0   
$EndComp
Text Notes 2400 5100 2    50   ~ 0
Power Input
Wire Wire Line
	9450 2550 9800 2550
$Comp
L power:+3.3V #PWR0117
U 1 1 6099E9B5
P 1850 7200
F 0 "#PWR0117" H 1850 7050 50  0001 C CNN
F 1 "+3.3V" V 1865 7328 50  0000 L CNN
F 2 "" H 1850 7200 50  0001 C CNN
F 3 "" H 1850 7200 50  0001 C CNN
	1    1850 7200
	0    -1   -1   0   
$EndComp
Text GLabel 1850 6700 0    50   Input ~ 0
MCU_SCL
Text GLabel 1850 6800 0    50   Input ~ 0
MCU_SDA
Wire Wire Line
	1850 6500 2350 6500
Wire Wire Line
	1850 6600 2350 6600
Wire Wire Line
	1850 6700 2100 6700
Wire Wire Line
	1850 6800 2250 6800
Wire Wire Line
	2100 6700 2100 6900
Connection ~ 2100 6700
Wire Wire Line
	2100 6700 2350 6700
Wire Wire Line
	2250 6800 2250 6900
Connection ~ 2250 6800
Wire Wire Line
	2250 6800 2350 6800
Wire Wire Line
	2100 7200 2250 7200
Wire Wire Line
	2100 7200 1850 7200
Connection ~ 2100 7200
$Comp
L Sensor_Temperature:DS18B20 U3
U 1 1 609AC62D
P 8900 4900
F 0 "U3" V 8533 4900 50  0000 C CNN
F 1 "DS18B20" V 8624 4900 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7900 4650 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 8750 5150 50  0001 C CNN
	1    8900 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 5500 8900 5500
Wire Wire Line
	8900 5500 8900 5200
Text GLabel 5950 3800 2    50   Input ~ 0
1-WIRE
Text GLabel 8400 5500 0    50   Input ~ 0
1-WIRE
Wire Wire Line
	8400 5500 8900 5500
Connection ~ 8900 5500
Wire Wire Line
	8900 5500 8900 5650
$Comp
L power:+3.3V #PWR0118
U 1 1 609C31D2
P 8450 5950
F 0 "#PWR0118" H 8450 5800 50  0001 C CNN
F 1 "+3.3V" V 8465 6078 50  0000 L CNN
F 2 "" H 8450 5950 50  0001 C CNN
F 3 "" H 8450 5950 50  0001 C CNN
	1    8450 5950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8450 5950 8900 5950
$Comp
L Device:CP C5
U 1 1 60958064
P 3250 2100
F 0 "C5" H 3365 2146 50  0000 L CNN
F 1 "1000uF" H 3365 2055 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-43_Kemet-X_Pad2.25x2.55mm_HandSolder" H 3288 1950 50  0001 C CNN
F 3 "~" H 3250 2100 50  0001 C CNN
	1    3250 2100
	1    0    0    -1  
$EndComp
Connection ~ 3250 1950
Wire Wire Line
	3250 1950 3450 1950
$Comp
L power:GND #PWR0119
U 1 1 60958422
P 3250 2250
F 0 "#PWR0119" H 3250 2000 50  0001 C CNN
F 1 "GND" H 3255 2077 50  0000 C CNN
F 2 "" H 3250 2250 50  0001 C CNN
F 3 "" H 3250 2250 50  0001 C CNN
	1    3250 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 6099A671
P 5350 4200
F 0 "#PWR0120" H 5350 3950 50  0001 C CNN
F 1 "GND" H 5350 4050 50  0000 C CNN
F 2 "" H 5350 4200 50  0001 C CNN
F 3 "" H 5350 4200 50  0001 C CNN
	1    5350 4200
	1    0    0    -1  
$EndComp
Connection ~ 2300 3600
Wire Wire Line
	8950 1950 8950 2150
Wire Wire Line
	7900 2150 8350 2150
Wire Wire Line
	8650 2150 8950 2150
Connection ~ 8950 2150
Wire Wire Line
	8950 2150 8950 2350
Wire Wire Line
	8450 5400 9200 5400
Wire Wire Line
	8450 5600 8600 5600
Wire Wire Line
	8600 4900 8600 5600
Connection ~ 8600 5600
Wire Wire Line
	8600 5600 9400 5600
Wire Wire Line
	9200 4900 9200 5400
Connection ~ 9200 5400
Wire Wire Line
	9200 5400 9400 5400
$Comp
L Connector_Generic:Conn_01x05 J4
U 1 1 60E0E019
P 5650 5400
F 0 "J4" H 5750 5400 50  0000 L CNN
F 1 "Conn_01x05" H 5750 5300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5650 5400 50  0001 C CNN
F 3 "~" H 5650 5400 50  0001 C CNN
	1    5650 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 60E0F653
P 5450 5300
F 0 "#PWR0113" H 5450 5050 50  0001 C CNN
F 1 "GND" V 5450 5100 50  0000 C CNN
F 2 "" H 5450 5300 50  0001 C CNN
F 3 "" H 5450 5300 50  0001 C CNN
	1    5450 5300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0115
U 1 1 60E0FE11
P 5450 5200
F 0 "#PWR0115" H 5450 5050 50  0001 C CNN
F 1 "+3.3V" V 5465 5328 50  0000 L CNN
F 2 "" H 5450 5200 50  0001 C CNN
F 3 "" H 5450 5200 50  0001 C CNN
	1    5450 5200
	0    -1   -1   0   
$EndComp
Text GLabel 5950 3300 2    50   Input ~ 0
TXD
Text GLabel 5950 3200 2    50   Input ~ 0
RXD
$Comp
L power:+3.3V #PWR0122
U 1 1 60E13D08
P 5350 1200
F 0 "#PWR0122" H 5350 1050 50  0001 C CNN
F 1 "+3.3V" H 5365 1373 50  0000 C CNN
F 2 "" H 5350 1200 50  0001 C CNN
F 3 "" H 5350 1200 50  0001 C CNN
	1    5350 1200
	1    0    0    -1  
$EndComp
Text Notes 5750 5000 2    50   ~ 0
Programming Port
Text GLabel 5450 5400 0    50   Input ~ 0
RXD
Text GLabel 5450 5500 0    50   Input ~ 0
TXD
$Comp
L MCU_Microchip_ATmega:ATmega328P-AU U4
U 1 1 6106ADCC
P 5350 2700
F 0 "U4" H 5350 950 50  0000 C CNN
F 1 "ATmega328P-AU" H 5350 850 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 5350 2700 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 5350 2700 50  0001 C CNN
	1    5350 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1200 5350 1200
Connection ~ 5350 1200
NoConn ~ 4750 1700
NoConn ~ 4750 1800
NoConn ~ 5950 2500
NoConn ~ 5950 2600
NoConn ~ 5950 2700
NoConn ~ 5950 3500
NoConn ~ 5950 3600
NoConn ~ 5950 3700
$Comp
L Device:C C6
U 1 1 6108EBE0
P 4300 1750
F 0 "C6" H 4415 1796 50  0000 L CNN
F 1 "0.1uF" H 4415 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4338 1600 50  0001 C CNN
F 3 "~" H 4300 1750 50  0001 C CNN
	1    4300 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1600 4300 1500
Wire Wire Line
	4300 1500 4750 1500
$Comp
L power:GND #PWR0114
U 1 1 61090DBE
P 4300 1900
F 0 "#PWR0114" H 4300 1650 50  0001 C CNN
F 1 "GND" H 4305 1727 50  0000 C CNN
F 2 "" H 4300 1900 50  0001 C CNN
F 3 "" H 4300 1900 50  0001 C CNN
	1    4300 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 6109140A
P 6750 2750
F 0 "R6" H 6680 2704 50  0000 R CNN
F 1 "10M" H 6680 2795 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6680 2750 50  0001 C CNN
F 3 "~" H 6750 2750 50  0001 C CNN
	1    6750 2750
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 61092262
P 7150 3000
F 0 "C7" V 6898 3000 50  0000 C CNN
F 1 "10uF" V 6989 3000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7188 2850 50  0001 C CNN
F 3 "~" H 7150 3000 50  0001 C CNN
	1    7150 3000
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 61092E2C
P 6750 3300
F 0 "SW1" V 6704 3448 50  0000 L CNN
F 1 "SW_Push" V 6795 3448 50  0000 L CNN
F 2 "" H 6750 3500 50  0001 C CNN
F 3 "~" H 6750 3500 50  0001 C CNN
	1    6750 3300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 61099DBD
P 6750 3500
F 0 "#PWR0121" H 6750 3250 50  0001 C CNN
F 1 "GND" H 6750 3350 50  0000 C CNN
F 2 "" H 6750 3500 50  0001 C CNN
F 3 "" H 6750 3500 50  0001 C CNN
	1    6750 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3000 6750 3000
Wire Wire Line
	6750 2900 6750 3000
Connection ~ 6750 3000
Wire Wire Line
	6750 3000 7000 3000
Wire Wire Line
	6750 3000 6750 3100
$Comp
L power:+3.3V #PWR0123
U 1 1 6109E8D3
P 6750 2600
F 0 "#PWR0123" H 6750 2450 50  0001 C CNN
F 1 "+3.3V" H 6765 2773 50  0000 C CNN
F 2 "" H 6750 2600 50  0001 C CNN
F 3 "" H 6750 2600 50  0001 C CNN
	1    6750 2600
	1    0    0    -1  
$EndComp
Text GLabel 7300 3000 2    50   Input ~ 0
RST
Text GLabel 5450 5600 0    50   Input ~ 0
RST
Text Label 6500 3000 0    50   ~ 0
RESET
Text GLabel 5950 2100 2    50   Input ~ 0
XTAL1
Text GLabel 5950 2200 2    50   Input ~ 0
XTAL2
Text GLabel 4100 2950 0    50   Input ~ 0
XTAL1
Text GLabel 4400 2950 2    50   Input ~ 0
XTAL2
$Comp
L power:GND #PWR?
U 1 1 610B62EF
P 4250 3150
F 0 "#PWR?" H 4250 2900 50  0001 C CNN
F 1 "GND" H 4255 2977 50  0000 C CNN
F 2 "" H 4250 3150 50  0001 C CNN
F 3 "" H 4250 3150 50  0001 C CNN
	1    4250 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:Resonator Y?
U 1 1 610B862B
P 4250 2950
F 0 "Y?" H 4250 3198 50  0000 C CNN
F 1 "Resonator" H 4250 3107 50  0000 C CNN
F 2 "Crystal:Resonator_SMD_muRata_CSTxExxV-3Pin_3.0x1.1mm_HandSoldering" H 4225 2950 50  0001 C CNN
F 3 "~" H 4225 2950 50  0001 C CNN
	1    4250 2950
	1    0    0    -1  
$EndComp
$EndSCHEMATC
