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
| `getcapture` / `setcapture 0\|1` | How pulses are counted. 1 = pin interrupt (default), 0 = timer poll |
| `getpulseguard` / `setpulseguard us` | Noise holdoff, 0-2000 us in steps of 10. Default 400 |
| `getcanreset` / `setcanreset 0\|1` | Act on a reset commanded over the bus. On by default |
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

## Wiring

The enclosure breaks out four GPIO on its terminal block. All four have an
external interrupt, so any of them can be used as the flow input.

| Terminal | `setflowpin` value | AVR pin | Interrupt |
| --- | --- | --- | --- |
| TX | 1 | PD3 | INT3 |
| RX | 0 (default) | PD2 | INT2 |
| SDA | 2 | PD1 | INT1 |
| SCL | 3 | PD0 | INT0 |

Using RX costs nothing: that is `Serial1`, and the console runs over USB CDC.

Pin 7 also has an external interrupt (INT4) and works as a flow input, but it
is not brought out to the terminal block on this enclosure.

## Pulse capture

`setcapture` selects how transducer pulses are counted. **The default is 1.**

**1 - pin interrupt (default).** Counts every falling edge in hardware,
accurate at any rate the transducer can produce. Edges closer together than the
noise holdoff (`setpulseguard`, 400 us by default) are rejected; see Electrical
noise below. Needs a flow pin with an external interrupt; the firmware falls
back to polling if the configured pin has none, and `status` shows that as
`Capture=1 active=0 (pin has no interrupt)`.

**0 - timer poll.** Samples the pin from the 1 kHz TIMER0 compare interrupt.
This is what earlier firmware did, and it is kept only as a fallback.

TIMER0 runs on a /64 prescaler, so the real sample rate is 976.6 Hz and the
fastest pulse train it can resolve is 488 Hz. At the FT60's 68000 pulses per
gallon that is **25.9 GPH**, against a transducer rated for 0.6 to 70 GPH:

| Fuel flow | Pulse rate | Polled counter |
| --- | --- | --- |
| 10 GPH | 189 Hz | accurate |
| 20 GPH | 378 Hz | accurate |
| 25.9 GPH | 488 Hz | limit |
| 28 GPH | 529 Hz | undercounts |
| 40 GPH | 756 Hz | undercounts |
| 51.7 GPH | 977 Hz | count collapses toward zero |
| 70 GPH | 1322 Hz | aliased, unusable |

It does not fail gracefully. Once the pulse rate reaches the sample rate the
samples land on the same phase every time and the count collapses. Because the
error grows with flow, no single K-factor can correct it - a K tuned at one
flow rate reads wrong at every other.

**The table assumes the transducer output is roughly square.** Polling only
registers a falling edge if the LOW phase spans a sample instant, so if the
FT60 emits a short fixed-width pulse instead, the polled counter was dropping
pulses at *every* flow rate, not just above 25.9 GPH, and the "accurate" rows
do not hold. Either way interrupt capture is correct and the empirical
recalibration below is what settles the number. To check: log the counter for
60 s at a steady idle flow in each mode. Equal counts confirm the assumption.

## Calibrating K

The FT60 is specified at 68000 pulses/gallon, and its datasheet notes that
installation can shift this, which is why K is configurable.

If K is set well below 68000 it was probably dialed down to compensate for
the polled counter losing pulses. With interrupt capture every pulse is
counted, so that K will now over-read; the firmware warns about this at boot.

To recalibrate:

1. `getk` and note the current value.
2. `setk 68000`, then restart.
3. `reset` at the start of a session with a known fuel load.
4. At the next fill, compare the reported fuel used against the gallons
   actually added.
5. If they differ, `setk` to `K_current x (reported / actual)`. Reporting
   18 gal when 21 went in gives 68000 x 18/21 = 58286.

Running `setk` marks K as deliberately chosen, which stops the recalibration
note appearing in `status`. That matters because a correctly recalibrated K
often lands outside the nominal band, as the example above does.

`status` reports the peak pulse rate seen since boot, with its GPH equivalent.

Two caveats. It is derived from *counted* pulses, so under timer polling it
reports what the counter managed rather than what the transducer produced -
it can never exceed 488 Hz there, and above 51.7 GPH it actually falls. It is
only a true reading of the transducer under interrupt capture. It is also
averaged over one-second sample intervals, so a short burst is reported as its
contribution to that second rather than its instantaneous rate; it
under-reports transients.

## Electrical noise

Interrupt capture counts every falling edge, which means it counts noise too.
Timer polling was an accidental 977 Hz low-pass filter; that filtering is gone,
so the failure mode changes from undercounting to potentially overcounting.

The holdoff (`setpulseguard`) is the control. It ignores any edge arriving less
than that many microseconds after the last accepted one, capping the countable
rate at `1 / guard`. The 400 us default counts to 2500 Hz, which is 132 GPH at
K=68000 - comfortably past the FT60's 70 GPH ceiling while rejecting anything
faster. `status` shows the resulting ceiling in GPH for the K in use, so a
transducer with a much higher K will show an obviously too-low number.

**The guard must stay below the shortest real pulse period**, or it discards
fuel. At K=68000 and 70 GPH that period is 756 us.

To find out whether you have a noise problem:

1. Ignition on, engine off, fuel pump not running. Watch `pulses=` in the
   serial output and `RejectedEdges` in `status`. Both should be completely
   static. Anything else is electrical noise, not fuel.
2. Engine idling, so the alternator is spinning. At 1-3 GPH expect roughly
   20-60 pulses/sec.
3. Switch loads on and off at idle. A jump in either counter identifies the
   coupling path.

`RejectedEdges` counts edges the holdoff threw away, which separates ringing on
a real pulse from spurious extra pulses - the raw count alone cannot.

If it is noisy, the fix is hardware, in order of value:

- **An external 1-2.2 kOhm pull-up** from signal to +5 V. The internal AVR
  pull-up is 20-50 kOhm, so the signal line is a high-impedance node running
  the length of the car. A 100 uA coupled spike into 30 kOhm develops 3 V and
  crosses the logic threshold; into 1.5 kOhm it develops 0.15 V and does
  nothing.
- **An RC filter at the board** - 1 kOhm in series, 10 nF to ground. The 10 us
  time constant is nothing against a ~378 us low phase but heavily attenuates
  spikes.
- **Shielded twisted pair**, shield grounded at the board end only. Grounding
  both ends creates a loop for alternator current.
- **Check the ground return.** A sensor sharing a ground path with alternator
  current is a classic source of offset.

A DMM on AC volts across the battery with the engine running is a quick check
on the alternator itself: under about 0.1 V AC is healthy, over 0.5 V AC
suggests a failed rectifier diode.

## Refuel detection

The tank-full line is a **level indication, not a refuel event**. On a real car
it reads asserted across a wide range of tank contents - measured high with the
tank at roughly two thirds - so it is high for most of a session. Treating
"high" as "just refueled" zeroed the counter at every pit stop.

So a refuel is only called when the line has first been **held low for 10
seconds** (the tank actually ran down past the switch), and then all three of
these hold **continuously for a further 10 seconds**:

- the tank-full signal is asserted
- more than 4 gallons have been used, so a top-up is not mistaken for a stint
- road speed is below 3 mph

Any of them lapsing restarts the clock. The low period is debounced for the
same reason the dwell is: a float switch that dips low for a single sample
under braking is no more trustworthy than one that dips high, and an
undebounced latch would let one such sample re-create the fault.

`status` shows `RefuelArmed=` so you can see whether the low period has been
served, and the `debug 1` frame log carries `armed=` alongside the inputs.

Consequences worth knowing:

- **A tank topped up before the level drops past the switch is not detected.**
  There is no way to tell that apart from sitting in the pits with a full tank,
  which is the fault being fixed. Reset manually after a splash-and-go.
- **A refuel done with the device powered off is not detected**, because the
  arming state is not persisted. Fuel used is restored from EEPROM, so the dash
  will read low for the rest of that stint. **Send a `reset` at the start of
  each session**, or after any refuel with the device off.

Auto-reset is also held off whenever no *state* frame (`0x100B0002`) has been
received for 5 seconds, so a dash that dies or is unplugged cannot leave a
stale tank-full reading driving the detector. Capacity frames alone do not
count - they carry neither speed nor the tank-full signal.

## Commanded resets

Byte 5 of `0x100B0002` asks the device to zero the fuel used. This is acted on
by default.

A dash with its own refuel detection can command a reset off a momentary blip
of the tank-full switch - fuel sloshing as the car comes to a stop is enough -
which zeroes the counter mid-session. `setcanreset 0` makes the device ignore
the command. The console `reset` still works, and refuel detection is
unaffected.

With it disabled, `status` reports how many *frames* carrying a set reset byte
have been ignored since boot - a dash holding the byte set adds one per frame,
so read it as "is the dash asking?" rather than as a count of averted resets.
Bit 5 of the diagnostics flags latches once any has been ignored, so the dash
asking is visible in a log without the console.

The setting takes effect immediately; no restart is needed.

Note that with both this and `AutoReset` enabled there are two independent
mechanisms that can zero the counter. Picking one is usually simpler.

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
0x100B0005: FuelConsLiters/Lap x 10000 (0, 2), then diagnostics, see below<br>

Receives:<br>
0x100B0001: FuelCapacity / 100 (0, 2), LastLapMs (2, 4), AutoReset (6, 1)<br>
0x100B0002: Speed / 10 (0, 2), FuelLevelGal / 100 (2, 2), FuelFull (4, 1), FuelReset (5, 1)<br>

In metric mode the volumes on 0x100B0003 and 0x100B0004 are liters, the rate on
0x100B0003 is cc/sec, and the received capacity, level and speed are liters and
km/h. 0x100B0005 is always liters per lap.

## Diagnostics

Bytes 2-7 of `0x100B0005` carry state the dash cannot see and cannot infer
from what it already sends. Those bytes were previously always zero, so
decoding them is optional and nothing existing is affected.

| Byte | Content |
| --- | --- |
| 2 | Flags: bit0 refuel armed, bit1 dwelling, bit2 interrupt capture, bit3 state frames fresh, bit4 CAN controller fault (debounced, not the raw latched register), bit5 a bus reset was ignored |
| 3 | Resets since boot, saturating at 255 |
| 4 | Last reset reason: 0 none, 1 commanded over the bus, 2 refuel detected, 3 console |
| 5-6 | Uptime in seconds, saturating at 65535 |
| 7 | Rejected pulse edges **since the last reset**, exact to 255 then pinned. Note this differs from byte 3, which is since boot |

Log these and a counter that drops can be attributed without a laptop:

- **fuel used drops and uptime restarts** - the device rebooted and restored a
  stale value from EEPROM. That is an electrical fault, not the detector.
  Without uptime this is indistinguishable from a reset.
- **fuel used drops, reason 2** - the refuel detector fired. The armed and
  dwelling flags, with the tank-full and speed values the dash already logs,
  show why.
- **fuel used drops, reason 1** - the dash commanded it over the bus; reason 3
  means somebody typed `reset` on the console.
- **rejected edges climbing** - electrical noise on the transducer line.

The last reason and a lifetime reset count are also kept in EEPROM and shown
by `status`, so a session can still be explained if the bus was not logged.

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

### Arduino IDE
Open the sketch and build for the CANBed Elite, selecting **Arduino Leonardo**
as the board - it is an ATmega32U4 at 16 MHz with the same bootloader.

Requires Longan Labs' MCP2515 library, the release that provides `mcp_can.h`.
Install "Longan Labs Arduino CAN Bus Library for MCP2515" through the Library
Manager. Note that the current upstream repository has renamed that header to
`mcp_canbus.h`, so a fresh clone from GitHub will not satisfy the include.

### PlatformIO
`platformio.ini` builds the same sources:

```
pio run                build
pio run -t upload      build and flash
pio device monitor     serial console
```

PlatformIO does not care that no `.ino` matches the folder name, so it opens
this directory without complaint. Two things in that file are load-bearing:
`src_dir` is set to the repository root, because `src/` here holds the pure
C++ core rather than the sketch and PlatformIO would otherwise compile the
core and never find `setup()`; and the CAN library is taken from the Arduino
sketchbook via `lib_extra_dirs` rather than fetched, for the header-name
reason above.

If PlatformIO cannot find the board when uploading, set `upload_port` - the
port number changes between the running sketch and the bootloader.

### Compile check without either IDE

```
bash tools/avr-build.sh
```

This drives the Arduino AVR toolchain directly and reports flash and RAM
usage. Set `ARDUINO15` and `SKETCHBOOK` if your Arduino install is not in the
default location.
