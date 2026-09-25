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
bool canResetEnabled = true;
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
// Which route asked. Conflating the two would leave a counter found at zero
// ambiguous between the dash and a crew member with a laptop, which is the
// exact question this instrumentation exists to answer.
static bool resetCameFromBus = false;
static uint32_t lastFrameMs = 0;
static bool haveFrame = false;
// Tracked separately from any-frame activity: the refuel detector reads
// fuelFull and speed, which only the state frame carries. Running it before
// one has arrived would feed it Parameters' constructed defaults, and since
// a low fuelFull is now the arming condition rather than an inert value, that
// could arm the detector off a signal the sender never sent.
static uint32_t lastStateFrameMs = 0;
static bool haveStateFrame = false;
static uint8_t canRecoveryAttempts = 0;
static uint8_t resetCount = 0;
// Frames carrying a set reset byte that were ignored. Counts frames rather
// than distinct commands - a dash holding the byte set adds one per frame -
// so read it as "is the dash asking?", not as a count of averted resets.
static uint16_t ignoredCanResetFrames = 0;
static uint8_t lastResetReason = fuel::RESET_NONE;

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  metricUnits = settings.metric();
  canResetEnabled = settings.canResetEnabled();
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
  // First boot after the upgrade: the capture byte has never been written, so
  // the device has just adopted the new interrupt default. Persist it, so a
  // later firmware changing the default - or a downgrade - cannot silently
  // move the device back to polling with a K meant for interrupt capture.
  //
  // What gets written is the resolved setting, never activeCaptureMode().
  // If the configured pin has no external interrupt the device is polling
  // right now, but that is a fact about this boot, not a preference; freezing
  // it would leave the unit polling for good once the pin was corrected, with
  // nothing left to say the upgrade had ever happened.
  if (settings.captureModeWasDefaulted()) {
    settings.setCaptureMode(settings.captureMode());
    Serial.print(F("Pulse capture defaulted to "));
    Serial.println(settings.captureMode());
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
      noteReset(resetCameFromBus ? fuel::RESET_COMMANDED : fuel::RESET_CONSOLE);
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
      noteReset(fuel::RESET_REFUEL);
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
  transmitFuelData(status, now);
}

// Record why the counter was zeroed, in RAM for the CAN diagnostics and in
// EEPROM so a session can still be explained afterwards if the bus was not
// being logged.
uint16_t ignoredCanResetFrameCount() {
  return ignoredCanResetFrames;
}

void noteReset(uint8_t reason) {
  lastResetReason = reason;
  if (resetCount < 255) resetCount++;
  settings.recordReset(reason);
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

// True while the sender is still supplying the signals the refuel detector
// reads. Deliberately about state frames alone: capacity frames keep arriving
// from some dashes even when the state stream has stopped, and stale speed or
// tank-full readings must not drive a reset.
bool busIsLive(uint32_t nowMs) {
  return haveStateFrame &&
         fuel::elapsed(nowMs, lastStateFrameMs) <= RX_TIMEOUT_MS;
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
      if ((uint32_t)id == fuel::CAN_ID_RX_STATE) {
        lastStateFrameMs = lastFrameMs;
        haveStateFrame = true;
      }
    }
    if (result.resetRequested) {
      if (canResetEnabled) {
        resetRequested = true;
        resetCameFromBus = true;
      } else if (ignoredCanResetFrames < 65535) {
        ignoredCanResetFrames++;
      }
    }

    if (debugCanFrames && result.accepted) {
      printCanFrame(id, len, buf, result.resetRequested);
    }
  }
}

void transmitFuelData(const fuel::FuelStatus& status, uint32_t nowMs) {
  // State the dash cannot see and cannot infer from what it already sends.
  fuel::Diagnostics diag;
  diag.refuelArmed = autoResetDetector.isArmed();
  diag.refuelDwelling = autoResetDetector.isDwelling();
  diag.interruptCapture = (activeCaptureMode() == fuel::CAPTURE_PIN_INTERRUPT);
  diag.stateFresh = busIsLive(nowMs);
  // The monitor's debounced view, not the raw register: the MCP2515 latches
  // receive-overflow bits until software clears them, so one transient
  // overflow would otherwise pin this high for the rest of the power cycle
  // and read in a log as a live fault.
  diag.canError = canFaultMonitor.isFaulted();
  diag.canResetIgnored = (ignoredCanResetFrames > 0);
  diag.resetCount = resetCount;
  diag.lastResetReason = lastResetReason;
  diag.uptimeMs = nowMs;
  diag.rejectedEdges = readRejectedEdges();

  fuel::TxFrames frames;
  fuel::encodeStatus(status, metricUnits, diag, frames);

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

void printCanFrame(unsigned long id, unsigned char len,
                   const unsigned char* buf, bool resetRequested) {
  Serial.print(F("rx id="));
  Serial.print(id, HEX);
  Serial.print(F(",len="));
  Serial.print(len);
  // Raw payload and the decoded reset request. Without these there is no way
  // to tell a reset commanded over the bus from one the device decided on,
  // which is the single most useful thing to know when the counter drops.
  Serial.print(F(",data="));
  for (unsigned char i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  Serial.print(F(",RESET="));
  Serial.print(resetRequested ? 1 : 0);
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
  Serial.print(params.fuelFull);
  // Behavior now depends on this too, so it has to be visible when
  // diagnosing from a serial capture.
  Serial.print(F(",armed="));
  Serial.println(autoResetDetector.isArmed());
}

// Asks the main loop to reset on its next pass. Called from the console.
void requestReset() {
  resetRequested = true;
  resetCameFromBus = false;
}
