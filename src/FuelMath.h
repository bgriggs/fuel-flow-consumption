// FuelMath.h - Pure, hardware-independent fuel computation logic.
//
// Nothing in this header touches the Arduino API, so every function and class
// here can be compiled and exercised on a host machine by the unit tests in
// test/. The .ino files are thin adapters that own the hardware (CAN, EEPROM,
// Serial, the pulse ISR) and delegate all decisions to this layer.
#ifndef FUEL_MATH_H
#define FUEL_MATH_H

#include <stdint.h>

namespace fuel {

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

// Milliseconds between `since` and `now`, correct across the ~49.7 day
// millis() rollover because unsigned wrap-around subtraction is well defined.
// Always route elapsed-time math through here rather than comparing timestamps
// directly.
inline uint32_t elapsed(uint32_t now, uint32_t since) {
  return now - since;
}

// ---------------------------------------------------------------------------
// Unit conversion
// ---------------------------------------------------------------------------

const float LITERS_PER_GALLON = 3.785411784f;
const float GALLONS_PER_LITER = 0.264172052f;
const float MILES_PER_KM = 0.621371192f;
// 1 gal/min = 3785.411784 cc / 60 s
const float CCSEC_PER_GALMIN = 63.09019640f;

inline float gallonsToLiters(float gallons) { return gallons * LITERS_PER_GALLON; }
inline float litersToGallons(float liters) { return liters * GALLONS_PER_LITER; }
inline float kphToMph(float kph) { return kph * MILES_PER_KM; }

// ---------------------------------------------------------------------------
// Safe numeric conversion
// ---------------------------------------------------------------------------

// Converting a float that is negative, NaN, or larger than the destination
// range to an unsigned integer is undefined behavior in C++, and on AVR it
// yields arbitrary bit patterns rather than a saturated value. Every float
// that reaches a CAN frame goes through these instead of a raw cast.
uint16_t toU16(float value);
uint32_t toU32(float value);

// True when `value` is NaN. Avoids pulling in <math.h> / <cmath>, whose
// isnan() spelling differs between the AVR and host toolchains.
inline bool isNotANumber(float value) { return value != value; }

// Clamp a float to [lo, hi], mapping NaN to `lo`.
float clampf(float value, float lo, float hi);

// ---------------------------------------------------------------------------
// Data carried over the bus
// ---------------------------------------------------------------------------

struct FuelStatus {
  uint32_t fuelPulses;
  float fuelUsedGals;
  float fuelRemainingGals;
  float fuelConsumptionGalMin;
  float fuelRemainingMins;
  float fuelConsumptionGalLap;
  int16_t lapsRemaining;

  FuelStatus()
    : fuelPulses(0), fuelUsedGals(0.0f), fuelRemainingGals(0.0f),
      fuelConsumptionGalMin(0.0f), fuelRemainingMins(0.0f),
      fuelConsumptionGalLap(0.0f), lapsRemaining(0) {}
};

struct Parameters {
  float capacityGals;
  uint32_t lastLapMs;
  float speedMph;
  float fuelLevelGals;
  bool fuelFull;
  bool autoReset;

  Parameters()
    : capacityGals(0.0f), lastLapMs(0), speedMph(0.0f), fuelLevelGals(0.0f),
      fuelFull(false), autoReset(false) {}
};

// Plausibility limits for values arriving from the bus. Anything outside these
// is treated as corrupt and discarded rather than propagated into the
// calculations, where it would produce absurd laps-remaining figures or
// divide-by-zero.
const float MAX_PLAUSIBLE_CAPACITY_GALS = 200.0f;
const float MAX_PLAUSIBLE_SPEED_MPH = 300.0f;
const uint32_t MIN_PLAUSIBLE_LAP_MS = 5000UL;      // 5 s
const uint32_t MAX_PLAUSIBLE_LAP_MS = 1800000UL;   // 30 min

// A lap time that could not be real - zero, or a sender reporting seconds
// where milliseconds are expected, or an all-ones field from a garbled frame.
inline bool isPlausibleLapMs(uint32_t lapMs) {
  return lapMs >= MIN_PLAUSIBLE_LAP_MS && lapMs <= MAX_PLAUSIBLE_LAP_MS;
}

}  // namespace fuel

#endif  // FUEL_MATH_H
