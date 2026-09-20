// FuelComputer.h - Turns a raw transducer pulse count into fuel used,
// consumption rate, endurance and laps remaining.
//
// Holds a rolling window of pulse samples and a short history of lap times.
// Entirely pure: the caller supplies "now" and the current pulse count, so the
// whole thing can be driven from a test at any speed.
#ifndef FUEL_COMPUTER_H
#define FUEL_COMPUTER_H

#include <stdint.h>
#include "FuelMath.h"

namespace fuel {

struct FuelSample {
  uint32_t timeMs;
  uint32_t pulses;
};

class FuelComputer {
public:
  // Consumption is averaged over a rolling window of (HISTORY_SAMPLES - 1)
  // intervals, so 60 samples one second apart span 59 s.
  //
  // The window length is set by how often update() is actually called, not by
  // SAMPLE_INTERVAL_MS alone: a sample is taken on the first call at or after
  // each interval. The firmware calls update() every 500 ms, so samples land
  // exactly 1000 ms apart and the window is 59 s.
  //
  // That matches the old firmware, whose 120-entry history had a 100 ms gate
  // but was likewise only called from the 500 ms status branch - giving
  // 119 x 500 ms = 59.5 s. Halving the entry count while doubling the spacing
  // keeps the same averaging period for 480 bytes instead of 960, which
  // matters on a part with 2.5 KB of RAM.
  static const uint8_t HISTORY_SAMPLES = 60;
  static const uint16_t SAMPLE_INTERVAL_MS = 1000;
  static const uint8_t MAX_LAPS = 3;

  // Endurance is meaningless below this burn rate (engine off, or a stalled
  // pulse train) and dividing by it produces absurd figures.
  static const uint16_t MIN_CONSUMPTION_MILLIGAL_MIN = 1;  // 0.001 gal/min

  // Display ceilings. These also keep the encoded CAN values inside uint16:
  // 999 min x 60 = 59940 s.
  static const uint16_t MAX_REMAINING_MINS = 999;
  static const uint16_t MAX_LAPS_REMAINING = 999;

  explicit FuelComputer(uint32_t k = 68000UL);

  // The transducer K-factor, in pulses per gallon. Values of zero are rejected
  // so fuelUsed can never divide by zero.
  void setK(uint32_t k);
  uint32_t k() const { return k_; }

  // Discard the rolling window and lap history. Does not touch the pulse
  // counter, which the caller owns.
  void reset();

  // Nominal window length, for reporting and tests. The real span depends on
  // the call cadence as described above.
  static const uint32_t NOMINAL_WINDOW_MS =
      (uint32_t)(HISTORY_SAMPLES - 1) * SAMPLE_INTERVAL_MS;

  // Feed a completed lap time. Duplicates of the previous value, zeroes and
  // implausible durations are ignored. Returns true if the lap was recorded.
  bool recordLap(uint32_t lapMs);

  // Sample the counter and recompute. Safe to call at any rate; samples are
  // only appended to the window every SAMPLE_INTERVAL_MS.
  FuelStatus update(uint32_t nowMs, uint32_t pulses, float capacityGals);

  // Exposed for tests.
  uint8_t sampleCount() const { return sampleCount_; }
  uint8_t lapCount() const { return lapCount_; }

private:
  void pushSample(uint32_t nowMs, uint32_t pulses);
  bool windowSpan(uint32_t& durationMs, uint32_t& pulseDelta) const;
  bool averageLapMs(float& avgMs) const;

  uint32_t k_;

  FuelSample samples_[HISTORY_SAMPLES];
  uint8_t sampleHead_;   // index the next sample is written to
  uint8_t sampleCount_;  // number of valid samples, saturating at HISTORY_SAMPLES
  uint32_t lastSampleMs_;
  bool haveSampled_;

  uint32_t lapMs_[MAX_LAPS];
  uint8_t lapHead_;
  uint8_t lapCount_;
  uint32_t lastLapMs_;
};

}  // namespace fuel

#endif  // FUEL_COMPUTER_H
