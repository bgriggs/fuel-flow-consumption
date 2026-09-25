// Firmware.h - State and functions shared between the .ino files.
//
// The Arduino build concatenates every .ino in the sketch folder into one
// translation unit, and it only generates forward declarations for functions,
// not for variables. Declaring the shared objects here means the code no
// longer depends on which file the IDE happens to place first.
//
// Only .ino files include this; it pulls in the Arduino API.
#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <Arduino.h>

#include "ArduinoEeprom.h"
#include "AutoReset.h"
#include "CanCodec.h"
#include "CommandParser.h"
#include "Config.h"
#include "FaultMonitor.h"
#include "FuelComputer.h"
#include "FuelMath.h"

// --- shared state, defined in fuel-usage.ino -------------------------------

extern fuel::ArduinoEeprom eepromStore;
extern fuel::Settings settings;
extern fuel::Parameters params;
extern fuel::FuelComputer fuelComputer;
extern fuel::AutoReset autoResetDetector;
extern fuel::FaultMonitor canFaultMonitor;

// Cached at boot so the hot path never re-reads EEPROM.
extern bool metricUnits;

// Whether a reset commanded over the bus is acted on. Cached at boot.
extern bool canResetEnabled;
uint16_t ignoredCanResetFrameCount();

// Per-frame receive logging. Off by default. Serial here is USB CDC rather
// than a UART, so the configured baud is ignored and throughput far exceeds
// any CAN rate; the exposure is that a host which has the port open but has
// stopped draining it blocks Serial.print(), and a line per frame makes that
// far more likely than the twice-a-second status line alone.
extern bool debugCanFrames;

// --- fuel-usage.ino --------------------------------------------------------

// Declared explicitly rather than relying on the IDE's prototype generator.
void serviceCan();
void serviceCanHealth(uint32_t nowMs);
bool busIsLive(uint32_t nowMs);
void recordLapTime();
void configureCanFilters();
void transmitFuelData(const fuel::FuelStatus& status, uint32_t nowMs);
void noteReset(uint8_t reason);
void printStatus(const fuel::FuelStatus& status);
void printCanFrame(unsigned long id, unsigned char len,
                   const unsigned char* buf, bool resetRequested);

// Asks the main loop to zero the fuel used on its next pass, so the reset
// happens at a known point rather than partway through a status cycle.
void requestReset();

// --- fuel-flow.ino ---------------------------------------------------------

void startFlowMeter();
void resetFuel();

// Pulse counter accessors. The counter is 32-bit and incremented from an
// interrupt, so on an 8-bit core it must never be read or written directly:
// a plain access can be split by the ISR and return a torn value.
uint32_t readFuelPulses();

// Which counting method actually started, which may differ from the stored
// setting if the configured pin has no external interrupt.
uint8_t activeCaptureMode();
uint16_t activePulseGuardUs();

// Edges discarded by the noise holdoff since boot. A count that rises with
// the engine off means electrical noise is reaching the input.
uint32_t readRejectedEdges();
void clearRejectedEdges();

// --- commands.ino ----------------------------------------------------------

void serviceSerial();
void handleCommand(const char* line);
void printSettings();
void printKAdviceIfNeeded();

#endif  // FIRMWARE_H
