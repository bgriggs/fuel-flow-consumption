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

void printSettings() {
  Serial.print(F("CAN Speed="));
  Serial.println(settings.canSpeed());
  Serial.print(F("Flow Sensor Pin="));
  Serial.println(settings.flowPin());
  Serial.print(F("MetricUnits="));
  Serial.println(settings.metric() ? 1 : 0);
  Serial.print(F("K="));
  Serial.println(settings.k());
  Serial.print(F("Capture="));
  Serial.print(settings.captureMode());
  Serial.print(F(" active="));
  Serial.println(activeCaptureMode());
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
  Serial.println(F("  setcapture 0|1    0=timer poll, 1=pin interrupt"));
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
      Serial.print(F("K="));
      Serial.println(settings.k());
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
      if (settings.captureMode() == fuel::CAPTURE_PIN_INTERRUPT) {
        Serial.println(F("Interrupt capture counts every pulse; K will need"));
        Serial.println(F("recalibrating if it was tuned against polling."));
      }
      printRestartNotice();
      break;

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
