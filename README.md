# Introduction
Measures fuel consumption using a Fuel Flow Transducer FT60 and sends data over a CAN bus. This runs on an Arduino CANBed Elite with CAN bus MCP2515 and MCP2551.

# Hardware
Arduino: CANBed Elite Atmega32U4 - MCP2515 and MCP2551 CAN bus chips https://www.seeedstudio.com/CANBed-Elite-Atmega32U4-p-5605.html
Fuel Flow Transducer: Electronics International Fuel Flow Transducer FT60 https://www.aircraftspruce.com/pages/in/fuelinstrumentation_fuelflow/eift60.php

# Usage
Use serial interface, such as in Arduino IDE, to send commands to the device.

## Commands
getcanspeed / setcanspeed xx<br>
Following are the speed options:<br>
-	#define CAN_5KBPS           1
-	#define CAN_10KBPS          2
-	#define CAN_20KBPS          3
-	#define CAN_25KBPS          4 
-	#define CAN_31K25BPS        5
-	#define CAN_33KBPS          6
-	#define CAN_40KBPS          7
-	#define CAN_50KBPS          8
-	#define CAN_80KBPS          9
-	#define CAN_83K3BPS         10
-	#define CAN_95KBPS          11
-	#define CAN_100KBPS         12
-	#define CAN_125KBPS         13
-	#define CAN_200KBPS         14
-	#define CAN_250KBPS         15
-	#define CAN_500KBPS         16
-	#define CAN_666KBPS         17
-	#define CAN_1000KBPS        18

getflowpin / setflowpin xx<br>    
RX is pin 0   (https://docs.arduino.cc/hacking/hardware/PinMapping32u4)

reset (Resets the fuel used)

## CAN IDs
Sends:<br>
0x100B0003: FuelPulses (0, 4), FuelUsedGal x 100 (4, 2), FuelConsGal/Min x 10000 (6, 2)<br>
0x100B0004: FuelRemGals (0, 2) x 100, FuelRemSecs (2, 2), FuelLapsRem (4, 2), FuelConsGal/Lap x 10000 (6, 2)<br>
0x100B0005: FuelConsLiters/Lap x 10000 (0, 2)<br>

Receives:<br>
0x100B0001: FuelCapacity / 100 (0, 2), LastLapMs (2, 4), AutoReset (6, 1)<br>
0x100B0002: Speed / 10 (0, 2), FuelLevelGal / 100 (2, 2), FuelFull (4, 1), FuelReset (5, 1)<br>



