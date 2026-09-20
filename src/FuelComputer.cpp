#include "FuelComputer.h"

namespace fuel {

FuelComputer::FuelComputer(uint32_t k)
  : k_(k > 0 ? k : 68000UL),
    sampleHead_(0),
    sampleCount_(0),
    lastSampleMs_(0),
    haveSampled_(false),
    lapHead_(0),
    lapCount_(0),
    lastLapMs_(0) {
  for (uint8_t i = 0; i < HISTORY_SAMPLES; i++) {
    samples_[i].timeMs = 0;
    samples_[i].pulses = 0;
  }
  for (uint8_t i = 0; i < MAX_LAPS; i++) lapMs_[i] = 0;
}

void FuelComputer::setK(uint32_t k) {
  if (k > 0) k_ = k;
}

void FuelComputer::reset() {
  sampleHead_ = 0;
  sampleCount_ = 0;
  haveSampled_ = false;
  lapHead_ = 0;
  lapCount_ = 0;
  // lastLapMs_ is deliberately kept. The caller re-offers the same lap time
  // from the bus on the next tick, and clearing it here would let that one lap
  // be counted a second time into the fresh average.
}

bool FuelComputer::recordLap(uint32_t lapMs) {
  if (lapMs == 0) return false;
  if (lapMs == lastLapMs_) return false;  // same lap reported repeatedly
  if (!isPlausibleLapMs(lapMs)) return false;

  lapMs_[lapHead_] = lapMs;
  lapHead_ = (uint8_t)((lapHead_ + 1) % MAX_LAPS);
  if (lapCount_ < MAX_LAPS) lapCount_++;
  lastLapMs_ = lapMs;
  return true;
}

void FuelComputer::pushSample(uint32_t nowMs, uint32_t pulses) {
  samples_[sampleHead_].timeMs = nowMs;
  samples_[sampleHead_].pulses = pulses;
  sampleHead_ = (uint8_t)((sampleHead_ + 1) % HISTORY_SAMPLES);
  if (sampleCount_ < HISTORY_SAMPLES) sampleCount_++;
}

// Span from the oldest to the newest sample. Only reports a span once the
// window is completely full, so the consumption figure always covers the same
// period and cannot swing wildly while the window fills.
bool FuelComputer::windowSpan(uint32_t& durationMs, uint32_t& pulseDelta) const {
  if (sampleCount_ < HISTORY_SAMPLES) return false;

  uint8_t newest = (uint8_t)((sampleHead_ + HISTORY_SAMPLES - 1) % HISTORY_SAMPLES);
  uint8_t oldest = sampleHead_;  // window is full, so head points at the oldest

  durationMs = elapsed(samples_[newest].timeMs, samples_[oldest].timeMs);
  if (durationMs == 0) return false;

  // The counter only ever climbs. A newest-below-oldest reading means the
  // counter was zeroed or a read tore, so publish nothing this round.
  if (samples_[newest].pulses < samples_[oldest].pulses) return false;
  pulseDelta = samples_[newest].pulses - samples_[oldest].pulses;
  return true;
}

bool FuelComputer::averageLapMs(float& avgMs) const {
  if (lapCount_ < MAX_LAPS) return false;
  uint32_t total = 0;
  uint8_t counted = 0;
  for (uint8_t i = 0; i < MAX_LAPS; i++) {
    if (lapMs_[i] > 0) {
      total += lapMs_[i];
      counted++;
    }
  }
  if (counted == 0) return false;
  avgMs = (float)total / (float)counted;
  return avgMs > 0.0f;
}

FuelStatus FuelComputer::update(uint32_t nowMs, uint32_t pulses,
                                float capacityGals) {
  if (!haveSampled_ || elapsed(nowMs, lastSampleMs_) >= SAMPLE_INTERVAL_MS) {
    pushSample(nowMs, pulses);
    lastSampleMs_ = nowMs;
    haveSampled_ = true;
  }

  FuelStatus s;
  s.fuelPulses = pulses;
  s.fuelUsedGals = (float)pulses / (float)k_;  // k_ is guaranteed non-zero

  if (capacityGals > 0.0f && !isNotANumber(capacityGals)) {
    s.fuelRemainingGals = capacityGals - s.fuelUsedGals;
    if (s.fuelRemainingGals < 0.0f) s.fuelRemainingGals = 0.0f;
  }

  uint32_t durationMs = 0;
  uint32_t pulseDelta = 0;
  if (windowSpan(durationMs, pulseDelta)) {
    float usedGals = (float)pulseDelta / (float)k_;
    float galPerSec = usedGals / ((float)durationMs / 1000.0f);
    s.fuelConsumptionGalMin = galPerSec * 60.0f;
    if (s.fuelConsumptionGalMin < 0.0f) s.fuelConsumptionGalMin = 0.0f;
  }

  const float minConsumption = (float)MIN_CONSUMPTION_MILLIGAL_MIN / 1000.0f;
  if (s.fuelRemainingGals > 0.0f && s.fuelConsumptionGalMin >= minConsumption) {
    s.fuelRemainingMins = clampf(s.fuelRemainingGals / s.fuelConsumptionGalMin,
                                 0.0f, (float)MAX_REMAINING_MINS);
  }

  float avgLapMs = 0.0f;
  if (averageLapMs(avgLapMs)) {
    float avgLapMins = avgLapMs / 60000.0f;
    if (avgLapMins > 0.0f) {
      s.fuelConsumptionGalLap = s.fuelConsumptionGalMin * avgLapMins;
      if (s.fuelRemainingMins > 0.0f) {
        float laps = s.fuelRemainingMins / avgLapMins;
        s.lapsRemaining = (int16_t)clampf(laps, 0.0f, (float)MAX_LAPS_REMAINING);
      }
    }
  }

  return s;
}

}  // namespace fuel
