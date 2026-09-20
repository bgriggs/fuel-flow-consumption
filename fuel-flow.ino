// Pulse capture from the fuel flow transducer.
//
// Two counting methods are available, selected by the 'setcapture' command:
//
//   0 - poll the input from the 1 kHz TIMER0 compare interrupt. This is the
//       original behavior and stays the default, because a unit already in
//       service has its K-factor calibrated against it. Sampling at 1 kHz
//       cannot resolve a pulse train above ~500 Hz, and an FT60 at 68000
//       pulses/gallon reaches 500 Hz at about 26 GPH, so above that this
//       undercounts and the shortfall grows with flow.
//
//   1 - count falling edges with a hardware interrupt. Accurate at any flow
//       rate the transducer supports, but K will need recalibrating after
//       switching. Needs a flow pin that has an external interrupt.

#include <Arduino.h>
#include <util/atomic.h>

#include "src/Firmware.h"

// Incremented from interrupt context; every access outside an ISR must be
// wrapped in ATOMIC_BLOCK. A 32-bit variable takes four instructions to read
// on this 8-bit core, and an interrupt landing between them yields a value
// that was never actually held - which is how a single stray reading could
// show a nonsensical jump in fuel used.
static volatile uint32_t fuelPulseCount = 0;
static volatile uint8_t lastFlowPinState = HIGH;
static volatile uint32_t lastPulseMicros = 0;

// Both are read from interrupt context. They are only written in
// startFlowMeter() before the interrupt is armed, so there is no race, but
// anything an ISR reads is qualified volatile here as a matter of course.
static volatile uint8_t flowPin = fuel::DEFAULT_FLOW_PIN;
static volatile uint8_t captureMode = fuel::CAPTURE_TIMER_POLL;

// Shortest credible gap between pulses, used to reject contact bounce and
// electrical noise in interrupt mode. An FT60 at its rated maximum is still
// well under 2 kHz, so 150 us discards nothing real.
static const uint16_t MIN_PULSE_INTERVAL_US = 150;

uint32_t readFuelPulses() {
  uint32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = fuelPulseCount; }
  return value;
}

void writeFuelPulses(uint32_t value) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { fuelPulseCount = value; }
}

uint8_t activeCaptureMode() {
  return captureMode;
}

// Interrupt-mode handler.
void onFlowPulse() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastPulseMicros) < MIN_PULSE_INTERVAL_US) return;
  lastPulseMicros = nowUs;
  fuelPulseCount++;
}

// Poll-mode handler: TIMER0 compare A fires once per millisecond. The Arduino
// core uses TIMER0 overflow for millis(), so compare A is free.
ISR(TIMER0_COMPA_vect) {
  uint8_t state = digitalRead(flowPin);
  if (state == lastFlowPinState) return;
  if (state == LOW) fuelPulseCount++;
  lastFlowPinState = state;
}

void startFlowMeter() {
  flowPin = settings.flowPin();
  captureMode = settings.captureMode();

  pinMode(flowPin, INPUT_PULLUP);
  lastFlowPinState = digitalRead(flowPin);
  lastPulseMicros = micros();

  // Restore the fuel used across a power cycle. Clamped so a corrupt stored
  // value cannot seed the counter with something absurd.
  float savedGals = settings.fuelUsedGallons();
  float savedPulses = savedGals * (float)fuelComputer.k();
  writeFuelPulses(fuel::toU32(fuel::clampf(savedPulses, 0.0f, 4.0e9f)));

  if (captureMode == fuel::CAPTURE_PIN_INTERRUPT) {
    int irq = digitalPinToInterrupt(flowPin);
    if (irq != NOT_AN_INTERRUPT) {
      attachInterrupt((uint8_t)irq, onFlowPulse, FALLING);
      Serial.print(F("Pulse capture=interrupt, pin="));
      Serial.println(flowPin);
      return;
    }
    // Configured for interrupt capture on a pin that cannot do it. Fall back
    // rather than silently counting nothing.
    captureMode = fuel::CAPTURE_TIMER_POLL;
    Serial.print(F("Pin has no interrupt, falling back to polling: pin="));
    Serial.println(flowPin);
  }

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  Serial.print(F("Pulse capture=timer poll, pin="));
  Serial.println(flowPin);
}

void resetFuel() {
  writeFuelPulses(0);
  fuelComputer.reset();
  // Make the refuel detector earn a fresh dwell, so the tank still reading
  // full immediately after a reset cannot trigger another one.
  autoResetDetector.rearm();
  settings.setFuelUsedGallons(0.0f);
}
