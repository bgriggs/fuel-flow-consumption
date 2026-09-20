# Introduction
Measures fuel consumption using a Fuel Flow Transducer FT60 and sends data over a CAN bus. This runs on an Arduino CANBed Elite with CAN bus MCP2515 and MCP2551.

# Hardware
Arduino: CANBed Elite Atmega32U4 - MCP2515 and MCP2551 CAN bus chips https://www.seeedstudio.com/CANBed-Elite-Atmega32U4-p-5605.html
Fuel Flow Transducer: Electronics International Fuel Flow Transducer FT60 https://www.aircraftspruce.com/pages/in/fuelinstrumentation_fuelflow/eift60.php

# Layout
| Path | Contents |
| --- | --- |
| `*.ino` | Hardware layer: CAN, pulse capture, serial console, timing |
| `src/` | Fuel calculations, refuel detection, wire format, settings. No Arduino dependencies |
| `test/` | Host unit tests for everything in `src/` |
| `tools/` | Build script for compiling the firmware outside the IDE |

Logic that makes decisions lives in `src/` and is covered by tests. The `.ino`
files only talk to hardware.

# Usage
Use serial interface, such as in Arduino IDE, to send commands to the device.

## Commands
Type `help` for the list. Commands are matched exactly and arguments are range
checked; a rejected value is reported and not stored.

| Command | Description |
| --- | --- |
| `help` | List commands |
| `status` | Show all settings |
| `getcanspeed` / `setcanspeed 1-18` | CAN bit rate, see table below |
| `getflowpin` / `setflowpin 0-30` | Transducer input pin. RX is pin 0 ([pin mapping](https://docs.arduino.cc/hacking/hardware/PinMapping32u4)) |
| `getmetric` / `setmetric 0\|1` | Report in liters and km/h instead of gallons and mph |
| `getk` / `setk 1000-1000000` | Transducer pulses per gallon (FT60 is around 68000) |
| `getcapture` / `setcapture 0\|1` | How pulses are counted, see below |
| `debug 0\|1` | Log every received CAN frame. Off by default |
| `reset` | Reset the fuel used |

Settings are stored in EEPROM and most need a restart to take effect.

Speed options for `setcanspeed`:

| Value | Rate | Value | Rate |
| --- | --- | --- | --- |
| 1 | CAN_5KBPS | 10 | CAN_83K3BPS |
| 2 | CAN_10KBPS | 11 | CAN_95KBPS |
| 3 | CAN_20KBPS | 12 | CAN_100KBPS |
| 4 | CAN_25KBPS | 13 | CAN_125KBPS |
| 5 | CAN_31K25BPS | 14 | CAN_200KBPS |
| 6 | CAN_33KBPS | 15 | CAN_250KBPS |
| 7 | CAN_40KBPS | 16 | CAN_500KBPS |
| 8 | CAN_50KBPS | 17 | CAN_666KBPS |
| 9 | CAN_80KBPS | 18 | CAN_1000KBPS |

## Pulse capture

`setcapture` selects how transducer pulses are counted.

**0 - timer poll (default).** The input is sampled from the 1 kHz TIMER0
compare interrupt. This cannot resolve a pulse train faster than about 500 Hz.
An FT60 at 68000 pulses/gallon passes 500 Hz at roughly 26 GPH, so above that
this undercounts, and the shortfall grows with flow rate. It is the default
because an in-service unit will have had its K-factor calibrated against it.

**1 - pin interrupt.** Counts falling edges in hardware, accurate at any flow
rate the transducer supports. Pulses closer together than 150 us are rejected
as noise. Requires a flow pin with an external interrupt (on the ATmega32U4
those are pins 0, 1, 2, 3 and 7); the firmware falls back to polling and says
so at boot if the configured pin has none.

**Switching to mode 1 will change the readings** on a unit whose K was tuned
against the polled counter. Recalibrate K afterwards.

## Refuel detection

When `AutoReset` is set on `0x100B0001`, fuel used is zeroed automatically once
all three of these have held **continuously for 10 seconds**:

- the tank-full signal is asserted
- more than 4 gallons have been used, so a top-up is not mistaken for a stint
- road speed is below 3 mph

Any of them lapsing restarts the 10 second clock. Requiring all three together
is what stops fuel sloshing onto the float switch at a stop, or a tank-full
line stuck asserted combined with one brief stop, from zeroing the counter
mid-session.

Auto-reset is also held off whenever nothing has been received on the bus for
5 seconds, so a dash that dies or is unplugged cannot leave a stale tank-full
reading driving the detector.

## CAN health

The MCP2515 latches error conditions, and a controller taken off the bus stays
off it. If the controller reports an error **and** the bus has gone silent for
5 seconds after previously working, the firmware re-initializes it, at most
three times per outage and only after the fault has persisted for 30 seconds.
A device powered up with the rest of the bus switched off has never received
anything, so it stays quiet rather than retrying.

## CAN IDs
Sends:<br>
0x100B0003: FuelPulses (0, 4), FuelUsedGal x 100 (4, 2), FuelConsGal/Min x 10000 (6, 2)<br>
0x100B0004: FuelRemGals (0, 2) x 100, FuelRemSecs (2, 2), FuelLapsRem (4, 2), FuelConsGal/Lap x 10000 (6, 2)<br>
0x100B0005: FuelConsLiters/Lap x 10000 (0, 2)<br>

Receives:<br>
0x100B0001: FuelCapacity / 100 (0, 2), LastLapMs (2, 4), AutoReset (6, 1)<br>
0x100B0002: Speed / 10 (0, 2), FuelLevelGal / 100 (2, 2), FuelFull (4, 1), FuelReset (5, 1)<br>

In metric mode the volumes on 0x100B0003 and 0x100B0004 are liters, the rate on
0x100B0003 is cc/sec, and the received capacity, level and speed are liters and
km/h. 0x100B0005 is always liters per lap.

Received frames are decoded only as far as the reported length. A field carried
by a byte that was not received keeps its previous value, so a short frame
cannot be read as a reset request or a tank-full signal.

# Building and testing

## Unit tests
The tests need only a C++ compiler; there is no hardware or Arduino dependency.

```
cd test
make
```

On Windows, run them under WSL or any environment with `g++` and `make`. To use
a different compiler:

```
make CXX=clang++
```

## Firmware
Open the sketch in the Arduino IDE and build for the CANBed Elite
(ATmega32U4). Requires the
[Longan Labs MCP2515 CAN library](https://github.com/Longan-Labs/Arduino_CAN_BUS_MCP2515).

To check that it still compiles without opening the IDE:

```
bash tools/avr-build.sh
```

This reproduces the IDE's build against the installed AVR toolchain and reports
flash and RAM usage. Set `ARDUINO15` and `SKETCHBOOK` if your Arduino install
is not in the default location.
