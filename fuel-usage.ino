// Fuel flow consumption monitor - main sketch.
//
// Counts pulses from an Electronics International FT60 fuel flow transducer,
// turns them into fuel used / burn rate / endurance / laps remaining, and
// publishes the result on a CAN bus. Runs on a CANBed Elite (ATmega32U4 with
// an MCP2515/MCP2551).
//
// The decision-making logic lives in src/ and is covered by the host test
// suite in test/. This file owns the hardware: CAN, timing and reporting.

#include <SPI.h>
#include <mcp_can.h>

#include "src/Firmware.h"

// --- hardware --------------------------------------------------------------

const int SPI_CS_PIN = 17;  // CANBed Elite
MCP_CAN CAN(SPI_CS_PIN);

// --- shared state (declared in src/Firmware.h) -----------------------------

fuel::ArduinoEeprom eepromStore;
fuel::Settings settings(eepromStore);
fuel::Parameters params;
fuel::FuelComputer fuelComputer;
fuel::AutoReset autoResetDetector;
fuel::FaultMonitor canFaultMonitor;
bool metricUnits = false;
bool debugCanFrames = false;

// --- timing ----------------------------------------------------------------

// How often the status is computed and published.
const uint16_t STATUS_PERIOD_MS = 500;
// EEPROM is rated for ~100,000 writes, so the fuel-used cell is saved sparingly.
const uint32_t SAVE_PERIOD_MS = 30000;
// Ignore repeat reset commands arriving inside this window.
const uint16_t RESET_DEBOUNCE_MS = 1000;
// Upper bound on frames drained per loop, so a stuck controller cannot wedge
// the main loop.
const uint8_t MAX_FRAMES_PER_LOOP = 16;
// How long the bus may go quiet before the received parameters are treated as
// stale and the refuel detector is held off.
const uint16_t RX_TIMEOUT_MS = 5000;
// A dead controller is re-initialized at most this many times per outage, so
// a car sitting with its dash off cannot put the device in a reinit loop.
const uint8_t MAX_CAN_RECOVERY_ATTEMPTS = 3;

static uint32_t lastStatusMs = 0;
static uint32_t lastSaveMs = 0;
static uint32_t lastResetMs = 0;
static bool haveReset = false;
static bool resetRequested = false;
static uint32_t lastFrameMs = 0;
static bool haveFrame = false;
static uint8_t canRecoveryAttempts = 0;

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  metricUnits = settings.metric();
  fuelComputer.setK(settings.k());
  startFlowMeter();

  uint8_t speed = settings.canSpeed();
  if (settings.canSpeedWasDefaulted()) {
    Serial.print(F("Stored CAN speed invalid, using default="));
    Serial.println(speed);
  }
  if (settings.flowPinWasDefaulted()) {
    Serial.print(F("Stored flow pin invalid, using default="));
    Serial.println(settings.flowPin());
  }
  if (settings.kWasDefaulted()) {
    Serial.print(F("Stored K invalid, using default="));
    Serial.println(settings.k());
  }

  Serial.print(F("CAN Speed="));
  Serial.println(speed);
  while (CAN_OK != CAN.begin(speed)) {
    Serial.println(F("CAN BUS FAIL!"));
    delay(1000);
  }

  configureCanFilters();

  Serial.println(F("CAN BUS OK!"));
  printSettings();
  Serial.println(F("Type 'help' for commands."));

  lastStatusMs = millis();
  lastSaveMs = lastStatusMs;
}

void loop() {
  serviceCan();
  serviceSerial();

  uint32_t now = millis();
  serviceCanHealth(now);

  // A reset asked for over the bus or the console. Debounced so a sender that
  // holds the flag set does not reset on every pass.
  if (resetRequested) {
    resetRequested = false;
    if (!haveReset || fuel::elapsed(now, lastResetMs) > RESET_DEBOUNCE_MS) {
      resetFuel();
      lastResetMs = now;
      haveReset = true;
      Serial.println(F("Fuel used reset."));
    }
  }

  if (fuel::elapsed(now, lastStatusMs) < STATUS_PERIOD_MS) return;
  lastStatusMs = now;

  recordLapTime();
  fuel::FuelStatus status = fuelComputer.update(now, readFuelPulses(),
                                                params.capacityGals);

  // Once the bus goes quiet, params holds whatever the sender last said
  // forever. A dash that dies or is unplugged while the tank reads full would
  // otherwise satisfy the refuel conditions a few seconds later.
  if (params.autoReset && busIsLive(now)) {
    if (autoResetDetector.update(now, params.fuelFull, status.fuelUsedGals,
                                 params.speedMph)) {
      resetFuel();
      lastResetMs = now;
      haveReset = true;
      Serial.println(F("Refuel detected, fuel used reset."));
      // Recompute so the frames about to go out describe the state after the
      // reset rather than the state that triggered it.
      status = fuelComputer.update(now, readFuelPulses(), params.capacityGals);
    }
  } else {
    // Do not let a part-accumulated dwell survive auto-reset being turned off,
    // or the bus dropping out mid-dwell.
    autoResetDetector.rearm();
  }

  if (fuel::elapsed(now, lastSaveMs) >= SAVE_PERIOD_MS) {
    settings.setFuelUsedGallons(status.fuelUsedGals);
    lastSaveMs = now;
  }

  printStatus(status);
  transmitFuelData(status);
}

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------

// Accept both inbound identifiers in both receive buffers. Previously mask 0
// was configured for standard identifiers this device never uses, so only
// RXB1 could hold a frame and back-to-back frames were dropped.
void configureCanFilters() {
  CAN.init_Mask(0, CAN_EXTID, 0x1FFFFFFFUL);
  CAN.init_Mask(1, CAN_EXTID, 0x1FFFFFFFUL);
  CAN.init_Filt(0, CAN_EXTID, fuel::CAN_ID_RX_CAPACITY);
  CAN.init_Filt(1, CAN_EXTID, fuel::CAN_ID_RX_STATE);
  CAN.init_Filt(2, CAN_EXTID, fuel::CAN_ID_RX_CAPACITY);
  CAN.init_Filt(3, CAN_EXTID, fuel::CAN_ID_RX_STATE);
  CAN.init_Filt(4, CAN_EXTID, fuel::CAN_ID_RX_CAPACITY);
  CAN.init_Filt(5, CAN_EXTID, fuel::CAN_ID_RX_STATE);
}

// The MCP2515 latches receive-buffer overflows and can be taken off the bus
// entirely by a burst of errors. Nothing used to look at that, so a controller
// in a bad state stayed silent until the car was power cycled. Recovery only
// runs after a fault has been continuously present for 30 seconds, which is
// far longer than the transient error-passive states a noisy bus produces.
void serviceCanHealth(uint32_t nowMs) {
  if (!canFaultMonitor.shouldCheck(nowMs)) return;

  // An error flag on its own is not enough to act on. The MCP2515 latches
  // receive-overflow bits until software clears them, so a single overflow
  // early in the device's life would otherwise look like a permanent fault and
  // trigger a pointless teardown. Only treat the controller as wedged when it
  // is reporting an error AND the bus has gone silent after previously
  // working - a device powered up with the rest of the bus off has never
  // received anything and should simply stay quiet, as the old firmware did.
  bool controllerError = (CAN.checkError() != CAN_OK);
  bool silent = haveFrame && fuel::elapsed(nowMs, lastFrameMs) > RX_TIMEOUT_MS;
  bool wedged = controllerError && silent;

  bool recover = canFaultMonitor.recordCheck(nowMs, wedged);

  if (canFaultMonitor.faultIsNew()) {
    Serial.println(F("CAN controller error and bus silent."));
  }

  if (!recover) return;

  if (canRecoveryAttempts >= MAX_CAN_RECOVERY_ATTEMPTS) return;
  canRecoveryAttempts++;

  Serial.print(F("Reinitializing CAN, attempt "));
  Serial.println(canRecoveryAttempts);
  if (CAN.begin(settings.canSpeed()) == CAN_OK) {
    configureCanFilters();
    Serial.println(F("CAN reinitialized."));
  } else {
    Serial.println(F("CAN reinitialize FAILED."));
  }
}

// True while the sender is still talking to us.
bool busIsLive(uint32_t nowMs) {
  return haveFrame && fuel::elapsed(nowMs, lastFrameMs) <= RX_TIMEOUT_MS;
}

// Drain every queued frame. The old code took at most one frame per pass and
// then slept 50 ms, so a sender faster than 20 frames/second filled the
// controller's buffers and the parameters went stale.
void serviceCan() {
  uint8_t drained = 0;
  while (CAN.checkReceive() == CAN_MSGAVAIL && drained < MAX_FRAMES_PER_LOOP) {
    drained++;

    unsigned long id = 0;
    unsigned char len = 0;
    unsigned char buf[8];
    // readMsgBufID only fills `len` bytes. Clearing first means a short frame
    // cannot leave stale stack data where a payload byte is expected.
    memset(buf, 0, sizeof(buf));

    if (CAN.readMsgBufID(&id, &len, buf) != CAN_OK) break;
    if (len > sizeof(buf)) len = sizeof(buf);

    fuel::DecodeResult result =
        fuel::decodeFrame((uint32_t)id, buf, len, metricUnits, params);
    if (result.accepted) {
      lastFrameMs = millis();
      haveFrame = true;
      // Traffic is flowing, so whatever went wrong before is over.
      canRecoveryAttempts = 0;
    }
    if (result.resetRequested) resetRequested = true;

    if (debugCanFrames && result.accepted) printCanFrame(id, len);
  }
}

void transmitFuelData(const fuel::FuelStatus& status) {
  fuel::TxFrames frames;
  fuel::encodeStatus(status, metricUnits, frames);

  CAN.sendMsgBuf(fuel::CAN_ID_TX_USAGE, CAN_EXTID, fuel::CAN_FRAME_LEN,
                 frames.usage);
  CAN.sendMsgBuf(fuel::CAN_ID_TX_REMAINING, CAN_EXTID, fuel::CAN_FRAME_LEN,
                 frames.remaining);
  CAN.sendMsgBuf(fuel::CAN_ID_TX_METRIC_LAP, CAN_EXTID, fuel::CAN_FRAME_LEN,
                 frames.metricLap);
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

// Feed the lap time through, and say something if it is being rejected -
// a sender reporting laps in seconds rather than milliseconds would otherwise
// leave laps remaining stuck at zero with no explanation.
void recordLapTime() {
  static uint32_t lastComplainedMs = 0;
  if (params.lastLapMs != 0 && !fuel::isPlausibleLapMs(params.lastLapMs)) {
    if (params.lastLapMs != lastComplainedMs) {
      lastComplainedMs = params.lastLapMs;
      Serial.print(F("Ignoring implausible lap time (ms): "));
      Serial.println(params.lastLapMs);
    }
    return;
  }
  fuelComputer.recordLap(params.lastLapMs);
}

void printStatus(const fuel::FuelStatus& status) {
  Serial.print(F("pulses="));
  Serial.print(status.fuelPulses);
  Serial.print(F(",galsUsed="));
  Serial.print(status.fuelUsedGals);
  Serial.print(F(",fuelRemaining="));
  Serial.print(status.fuelRemainingGals);
  Serial.print(F(",consGalMin="));
  Serial.print(status.fuelConsumptionGalMin);
  Serial.print(F(",remMins="));
  Serial.print(status.fuelRemainingMins);
  Serial.print(F(",lapsRem="));
  Serial.println(status.lapsRemaining);
}

void printCanFrame(unsigned long id, unsigned char len) {
  Serial.print(F("rx id="));
  Serial.print(id, HEX);
  Serial.print(F(",len="));
  Serial.print(len);
  Serial.print(F(",cap="));
  Serial.print(params.capacityGals);
  Serial.print(F(",lastLapMs="));
  Serial.print(params.lastLapMs);
  Serial.print(F(",autoReset="));
  Serial.print(params.autoReset);
  Serial.print(F(",speed="));
  Serial.print(params.speedMph);
  Serial.print(F(",fuelLevel="));
  Serial.print(params.fuelLevelGals);
  Serial.print(F(",fuelFull="));
  Serial.println(params.fuelFull);
}

// Asks the main loop to reset on its next pass. Called from the console.
void requestReset() {
  resetRequested = true;
}
