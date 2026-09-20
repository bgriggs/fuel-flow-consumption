// Serial console.
//
// Reads a line a character at a time. The old code called Serial.readString(),
// which blocks for the full one second stream timeout on every command and
// allocates a String on a heap that has very little room to spare on a
// 2.5 KB part. During that second no CAN frames were drained.

#include <Arduino.h>

#include "src/Firmware.h"

static const uint8_t MAX_LINE = 40;
// The IDE serial monitor can be set to send no line ending, so a line is also
// dispatched once input has been quiet for this long.
static const uint16_t LINE_IDLE_MS = 120;

static char lineBuf[MAX_LINE];
static uint8_t lineLen = 0;
static bool lineOverflow = false;
static uint32_t lastCharMs = 0;

static void dispatchLine() {
  lineBuf[lineLen] = '\0';
  if (lineOverflow) {
    Serial.println(F("Command too long."));
  } else if (lineLen > 0) {
    handleCommand(lineBuf);
  }
  lineLen = 0;
  lineOverflow = false;
}

void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastCharMs = millis();

    if (c == '\n' || c == '\r') {
      if (lineLen > 0 || lineOverflow) dispatchLine();
      continue;
    }
    if (lineLen < MAX_LINE - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // Keep consuming to the end of the line, but remember it was truncated
      // so a partial command is never executed.
      lineOverflow = true;
    }
  }

  if ((lineLen > 0 || lineOverflow) &&
      fuel::elapsed(millis(), lastCharMs) > LINE_IDLE_MS) {
    dispatchLine();
  }
}

static void reportInvalidArg(const __FlashStringHelper* what) {
  Serial.print(F("Invalid "));
  Serial.print(what);
  Serial.println(F(". Value not applied."));
}

static void printRestartNotice() {
  Serial.println(F("Restart to apply changes."));
}

// The countable ceiling implied by the stored holdoff and K. Printed whenever
// either changes, because it is their combination that decides whether real
// flow gets discarded, and a K large enough to matter is easy to set without
// realizing.
static void printGuardCeiling() {
  uint16_t us = settings.pulseGuardUs();
  uint16_t hz = fuel::countableRateHz(us);
  if (hz == 0) {
    Serial.println(F("No holdoff: every edge is counted, including noise."));
    return;
  }
  float gph = (float)hz * 3600.0f / (float)settings.k();
  Serial.print(F("Counts to "));
  Serial.print(hz);
  Serial.print(F(" Hz = "));
  Serial.print(gph);
  Serial.println(F(" GPH"));
  // An FT60 tops out at 70 GPH, so a ceiling near that is already too close.
  if (gph < 100.0f) {
    Serial.println(F("WARNING: that may clip real flow. Lower the guard"));
    Serial.println(F("with 'setpulseguard' or check K."));
  }
}

void printSettings() {
  Serial.print(F("CAN Speed="));
  Serial.println(settings.canSpeed());
  Serial.print(F("Flow Sensor Pin="));
  Serial.println(settings.flowPin());
  Serial.print(F("MetricUnits="));
  Serial.println(settings.metric() ? 1 : 0);
  Serial.print(F("K="));
  Serial.print(settings.k());
  if (settings.k() != fuelComputer.k()) {
    Serial.print(F(" (active="));
    Serial.print(fuelComputer.k());
    Serial.print(F(", restart to apply)"));
  }
  Serial.println();
  printKAdviceIfNeeded();
  Serial.print(F("Capture="));
  Serial.print(settings.captureMode());
  Serial.print(F(" active="));
  Serial.print(activeCaptureMode());
  if (settings.captureMode() != activeCaptureMode()) {
    Serial.print(F(" (pin has no interrupt)"));
  }
  Serial.println();
  // The holdoff and the rejected-edge count only mean anything under
  // interrupt capture: in poll mode the handler is never attached, so the
  // count is structurally zero and the ceiling is 488 Hz, not this one.
  if (activeCaptureMode() == fuel::CAPTURE_PIN_INTERRUPT) {
    Serial.print(F("PulseGuard="));
    Serial.print(activePulseGuardUs());
    if (settings.pulseGuardUs() != activePulseGuardUs()) {
      Serial.print(F(" us (stored="));
      Serial.print(settings.pulseGuardUs());
      Serial.print(F(", restart to apply)"));
    } else {
      Serial.print(F(" us"));
    }
    Serial.println();
    printGuardCeiling();

    Serial.print(F("RejectedEdges="));
    Serial.print(readRejectedEdges());
    Serial.println(F(" (noise; should not rise with the engine off)"));
  } else {
    Serial.println(F("PulseGuard=n/a while polling"));
    Serial.println(F("Polling counts to 488 Hz = 25.9 GPH at K=68000"));
  }

  Serial.print(F("PeakPulseRate="));
  Serial.print(fuelComputer.peakPulsesPerSec());
  Serial.print(F(" Hz = "));
  Serial.print((float)fuelComputer.peakPulsesPerSec() * 3600.0f /
               (float)fuelComputer.k());
  Serial.print(F(" GPH, since reset"));
  if (activeCaptureMode() == fuel::CAPTURE_TIMER_POLL) {
    // In poll mode this is the rate the counter managed, not the rate the
    // transducer produced, and polling caps it at 488 Hz regardless.
    Serial.println(F(" (COUNTED rate; polling cannot exceed 488 Hz)"));
  } else {
    Serial.println();
  }
}

// Shown wherever settings are printed, not just at boot: on a 32U4 the USB
// serial port drops everything written before the host opens it, so a boot
// message is rarely seen.
void printKAdviceIfNeeded() {
  if (activeCaptureMode() != fuel::CAPTURE_PIN_INTERRUPT) return;
  if (fuel::isNearNominalK(settings.k())) return;
  // Any deliberate setk counts as an answer. Without this the advice would
  // nag forever at someone who had just recalibrated correctly, since a right
  // answer frequently lands outside the FT60 band.
  if (settings.kAcknowledged()) return;
  Serial.print(F("  note: FT60 nominal K is "));
  Serial.print(fuel::DEFAULT_K);
  Serial.println(F("; a K tuned against the old"));
  Serial.println(F("  polled counter will over-read now. See README."));
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  status            show all settings"));
  Serial.println(F("  getcanspeed"));
  Serial.println(F("  setcanspeed 1-18  see README for the speed table"));
  Serial.println(F("  getflowpin"));
  Serial.println(F("  setflowpin 0-30"));
  Serial.println(F("  getmetric"));
  Serial.println(F("  setmetric 0|1"));
  Serial.println(F("  getk"));
  Serial.println(F("  setk 1000-1000000 transducer pulses per gallon"));
  Serial.println(F("  getcapture"));
  Serial.println(F("  setcapture 0|1    1=pin interrupt (default), 0=poll"));
  Serial.println(F("  getpulseguard"));
  Serial.println(F("  setpulseguard us  noise holdoff, 0-2000 (default 400)"));
  Serial.println(F("  debug 0|1         log every received CAN frame"));
  Serial.println(F("  reset             zero the fuel used"));
}

void handleCommand(const char* line) {
  fuel::Command cmd = fuel::parseCommand(line);

  Serial.print(F("Received: "));
  Serial.println(line);

  switch (cmd.id) {
    case fuel::CMD_HELP:
      printHelp();
      break;

    case fuel::CMD_STATUS:
      printSettings();
      break;

    case fuel::CMD_GET_CAN_SPEED:
      Serial.print(F("CAN Speed="));
      Serial.println(settings.canSpeed());
      break;

    case fuel::CMD_SET_CAN_SPEED:
      // Range checked before it is stored, instead of writing whatever the
      // text parsed to and failing to start the controller on next boot.
      if (!cmd.argValid || !fuel::isValidCanSpeed((uint8_t)cmd.arg) ||
          cmd.arg != (int32_t)(uint8_t)cmd.arg) {
        reportInvalidArg(F("CAN speed (expected 1-18)"));
        break;
      }
      settings.setCanSpeed((uint8_t)cmd.arg);
      Serial.print(F("Updated CAN Speed="));
      Serial.println(settings.canSpeed());
      printRestartNotice();
      break;

    case fuel::CMD_GET_FLOW_PIN:
      Serial.print(F("Flow Sensor Pin="));
      Serial.println(settings.flowPin());
      break;

    case fuel::CMD_SET_FLOW_PIN:
      if (!cmd.argValid || cmd.arg < 0 || cmd.arg > (int32_t)fuel::MAX_FLOW_PIN) {
        reportInvalidArg(F("flow pin (expected 0-30)"));
        break;
      }
      settings.setFlowPin((uint8_t)cmd.arg);
      Serial.print(F("Flow Sensor Pin="));
      Serial.println(settings.flowPin());
      printRestartNotice();
      break;

    case fuel::CMD_GET_METRIC:
      Serial.print(F("MetricUnits="));
      Serial.println(settings.metric() ? 1 : 0);
      break;

    case fuel::CMD_SET_METRIC:
      if (!cmd.argValid || cmd.arg < 0 || cmd.arg > 1) {
        reportInvalidArg(F("metric flag (expected 0 or 1)"));
        break;
      }
      settings.setMetric(cmd.arg != 0);
      Serial.print(F("MetricUnits="));
      Serial.println(settings.metric() ? 1 : 0);
      printRestartNotice();
      break;

    case fuel::CMD_GET_K:
      Serial.print(F("K="));
      Serial.println(settings.k());
      break;

    case fuel::CMD_SET_K:
      // A K of zero used to be accepted and then divided by.
      if (!cmd.argValid || cmd.arg < 0 || !fuel::isValidK((uint32_t)cmd.arg)) {
        reportInvalidArg(F("K (expected 1000-1000000)"));
        break;
      }
      settings.setK((uint32_t)cmd.arg);
      settings.setKAcknowledged();
      Serial.print(F("K="));
      Serial.println(settings.k());
      // K and the holdoff together set the countable ceiling. A large K can
      // push the transducer's own output past it, which then looks exactly
      // like electrical noise in RejectedEdges.
      printGuardCeiling();
      printRestartNotice();
      break;

    case fuel::CMD_GET_CAPTURE:
      Serial.print(F("Capture="));
      Serial.print(settings.captureMode());
      Serial.print(F(" active="));
      Serial.println(activeCaptureMode());
      break;

    case fuel::CMD_SET_CAPTURE:
      if (!cmd.argValid || cmd.arg < 0 ||
          !fuel::isValidCaptureMode((uint8_t)cmd.arg)) {
        reportInvalidArg(F("capture mode (expected 0 or 1)"));
        break;
      }
      settings.setCaptureMode((uint8_t)cmd.arg);
      Serial.print(F("Capture="));
      Serial.println(settings.captureMode());
      if (settings.captureMode() == fuel::CAPTURE_TIMER_POLL) {
        Serial.println(F("WARNING: polling cannot resolve pulses above"));
        Serial.println(F("488 Hz, which is only 25.9 GPH at K=68000."));
      }
      printRestartNotice();
      break;

    case fuel::CMD_GET_PULSE_GUARD:
      // Stored and active can differ until a restart, so report both rather
      // than contradicting what setpulseguard just echoed.
      Serial.print(F("PulseGuard="));
      Serial.print(settings.pulseGuardUs());
      Serial.print(F(" us (active="));
      Serial.print(activePulseGuardUs());
      Serial.println(F(")"));
      break;

    case fuel::CMD_SET_PULSE_GUARD: {
      // Entered in microseconds; stored in units of 10 us.
      if (!cmd.argValid || cmd.arg < 0 ||
          cmd.arg > (int32_t)(fuel::MAX_PULSE_GUARD_UNITS *
                              fuel::PULSE_GUARD_UNIT_US) ||
          (cmd.arg % fuel::PULSE_GUARD_UNIT_US) != 0) {
        reportInvalidArg(F("pulse guard (expected 0-2000 us, multiple of 10)"));
        break;
      }
      settings.setPulseGuardUnits(
          (uint8_t)(cmd.arg / fuel::PULSE_GUARD_UNIT_US));
      Serial.print(F("PulseGuard="));
      Serial.print(settings.pulseGuardUs());
      Serial.println(F(" us"));
      printGuardCeiling();
      printRestartNotice();
      break;
    }

    case fuel::CMD_SET_DEBUG:
      if (!cmd.argValid || cmd.arg < 0 || cmd.arg > 1) {
        reportInvalidArg(F("debug flag (expected 0 or 1)"));
        break;
      }
      debugCanFrames = (cmd.arg != 0);
      Serial.print(F("debug="));
      Serial.println(debugCanFrames ? 1 : 0);
      break;

    case fuel::CMD_RESET:
      requestReset();
      break;

    case fuel::CMD_NONE:
      break;

    case fuel::CMD_UNKNOWN:
    default:
      Serial.println(F("Unknown command. Type 'help'."));
      break;
  }
}
