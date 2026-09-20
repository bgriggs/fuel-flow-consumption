// Pulse capture from the fuel flow transducer.
//
// Two counting methods are available, selected by the 'setcapture' command:
//
//   1 - count falling edges with a hardware interrupt (the default). Accurate
//       at any rate the transducer can produce. Edges arriving closer together
//       than the configurable holdoff ('setpulseguard') are rejected as noise.
//
//   0 - poll the input from the 1 kHz TIMER0 compare interrupt. What earlier
//       firmware did, kept as a fallback for a flow pin with no external
//       interrupt. TIMER0 runs on a /64 prescaler, so this samples at 976.6 Hz
//       and cannot resolve a pulse train above 488 Hz - only 25.9 GPH at the
//       FT60's 68000 pulses/gallon, against a transducer rated to 70 GPH. It
//       does not degrade gracefully either: near 977 Hz (51.7 GPH) the samples
//       land on the same phase each time and the count collapses toward zero.
//
// Switching between the two changes the counts at high flow, so K has to be
// right for the mode in use. Below 25.9 GPH both count identically, because
// both count falling edges.

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

// flowPin is read from interrupt context. It is only written in
// startFlowMeter() before the interrupt is armed, so there is no race, but
// anything an ISR reads is qualified volatile as a matter of course.
static volatile uint8_t flowPin = fuel::DEFAULT_FLOW_PIN;
static uint8_t captureMode = fuel::DEFAULT_CAPTURE_MODE;

// Shortest accepted gap between pulses, the only noise control the firmware
// has in interrupt mode. Configurable, because the right value depends on the
// transducer's K-factor and on how noisy the installation is; see Config.h.
//
// Read from interrupt context. Safe only because the single write happens in
// startFlowMeter() before attachInterrupt() arms the handler - 'setpulseguard'
// deliberately writes EEPROM and asks for a restart rather than applying live.
// Making it apply live would need the write wrapped in ATOMIC_BLOCK, since a
// 16-bit store from the main loop can tear against the ISR read.
static volatile uint16_t minPulseIntervalUs =
    (uint16_t)(fuel::DEFAULT_PULSE_GUARD_UNITS * fuel::PULSE_GUARD_UNIT_US);

// Edges thrown away by the holdoff. Ringing on a real pulse and a coupled
// spike train both show up here, so a count that climbs while the engine is
// off is direct evidence of electrical noise on the signal line.
static volatile uint32_t rejectedEdges = 0;

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

uint16_t activePulseGuardUs() {
  return minPulseIntervalUs;
}

uint32_t readRejectedEdges() {
  uint32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = rejectedEdges; }
  return value;
}

void clearRejectedEdges() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rejectedEdges = 0; }
}

// Interrupt-mode handler. A rejected edge deliberately does not refresh
// lastPulseMicros, so a noise source faster than the holdoff cannot hold the
// counter off indefinitely.
void onFlowPulse() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastPulseMicros) < minPulseIntervalUs) {
    rejectedEdges++;
    return;
  }
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
  minPulseIntervalUs = settings.pulseGuardUs();

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
  // The diagnostics describe a stint, so they start over with it. Otherwise
  // a peak or a noise count from three sessions ago keeps being reported.
  fuelComputer.clearPeakPulseRate();
  clearRejectedEdges();
  // Make the refuel detector earn a fresh dwell, so the tank still reading
  // full immediately after a reset cannot trigger another one.
  autoResetDetector.rearm();
  settings.setFuelUsedGallons(0.0f);
}
