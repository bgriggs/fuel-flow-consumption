#include "test_framework.h"
#include "../src/FuelComputer.h"

using namespace fuel;

namespace {

// K chosen so the arithmetic is exact: 60000 pulses/gal means 1 gal/min is
// 1000 pulses/sec, i.e. 1000 pulses per 1000 ms sample.
const uint32_t K = 60000;
const uint32_t PULSES_PER_SAMPLE_AT_1GPM =
    (uint32_t)FuelComputer::SAMPLE_INTERVAL_MS;

// Drive the computer at its natural sample rate with a constant flow, and
// return the final status.
FuelStatus runSteadyFlow(FuelComputer& fc, uint8_t samples,
                         uint32_t pulsesPerSample, float capacityGals,
                         uint32_t startMs = 0, uint32_t* pulsesOut = 0) {
  uint32_t pulses = 0;
  FuelStatus s;
  for (uint8_t i = 0; i < samples; i++) {
    uint32_t t = startMs + (uint32_t)i * FuelComputer::SAMPLE_INTERVAL_MS;
    s = fc.update(t, pulses, capacityGals);
    pulses += pulsesPerSample;
  }
  if (pulsesOut) *pulsesOut = pulses;
  return s;
}

}  // namespace

TEST(FuelComputer, fuel_used_is_pulses_over_k) {
  FuelComputer fc(K);
  FuelStatus s = fc.update(0, 30000, 0.0f);
  CHECK_NEAR(s.fuelUsedGals, 0.5f, 1e-6);
  CHECK_EQ(s.fuelPulses, 30000u);
}

// A zero K-factor reached the divide in the old code straight from EEPROM or
// from "setk 0", producing inf and then garbage in every derived figure.
TEST(FuelComputer, rejects_zero_k) {
  FuelComputer fc(0);
  CHECK(fc.k() > 0u);
  fc.setK(0);
  CHECK(fc.k() > 0u);
  FuelStatus s = fc.update(0, 1000, 0.0f);
  CHECK(!isNotANumber(s.fuelUsedGals));
  CHECK(s.fuelUsedGals < 1000.0f);
}

TEST(FuelComputer, remaining_is_capacity_less_used_and_never_negative) {
  FuelComputer fc(K);
  FuelStatus s = fc.update(0, 60000, 10.0f);  // 1 gal used of 10
  CHECK_NEAR(s.fuelRemainingGals, 9.0f, 1e-5);

  s = fc.update(250, 60000UL * 20UL, 10.0f);  // 20 gal used of 10
  CHECK_NEAR(s.fuelRemainingGals, 0.0f, 1e-6);
}

TEST(FuelComputer, remaining_is_zero_when_capacity_unknown) {
  FuelComputer fc(K);
  FuelStatus s = fc.update(0, 60000, 0.0f);
  CHECK_NEAR(s.fuelRemainingGals, 0.0f, 1e-6);
  CHECK_NEAR(s.fuelRemainingMins, 0.0f, 1e-6);
}

TEST(FuelComputer, consumption_is_withheld_until_the_window_is_full) {
  FuelComputer fc(K);
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES - 1,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f);
  CHECK_EQ(fc.sampleCount(), (uint8_t)(FuelComputer::HISTORY_SAMPLES - 1));
  CHECK_NEAR(s.fuelConsumptionGalMin, 0.0f, 1e-6);
}

TEST(FuelComputer, computes_steady_consumption_rate) {
  FuelComputer fc(K);
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f);
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-4);
}

TEST(FuelComputer, window_slides_and_tracks_a_rate_change) {
  FuelComputer fc(K);
  uint32_t pulses = 0;
  uint32_t t = 0;
  // Fill at 1 gal/min.
  for (uint8_t i = 0; i < FuelComputer::HISTORY_SAMPLES; i++) {
    fc.update(t, pulses, 20.0f);
    pulses += PULSES_PER_SAMPLE_AT_1GPM;
    t += FuelComputer::SAMPLE_INTERVAL_MS;
  }
  // Then run long enough at 2 gal/min to flush the window completely.
  FuelStatus s;
  for (uint8_t i = 0; i < FuelComputer::HISTORY_SAMPLES; i++) {
    s = fc.update(t, pulses, 20.0f);
    pulses += PULSES_PER_SAMPLE_AT_1GPM * 2;
    t += FuelComputer::SAMPLE_INTERVAL_MS;
  }
  CHECK_NEAR(s.fuelConsumptionGalMin, 2.0f, 1e-3);
}

// resetFuel() zeroes the counter. Until the window refills, the newest reading
// is below the oldest; subtracting them as unsigned would produce a ~4 billion
// pulse delta and an absurd burn rate.
TEST(FuelComputer, counter_going_backwards_does_not_produce_a_huge_rate) {
  FuelComputer fc(K);
  uint32_t pulses = 0;
  uint32_t t = 0;
  for (uint8_t i = 0; i < FuelComputer::HISTORY_SAMPLES; i++) {
    fc.update(t, pulses, 20.0f);
    pulses += PULSES_PER_SAMPLE_AT_1GPM;
    t += FuelComputer::SAMPLE_INTERVAL_MS;
  }
  // Counter zeroed without the window being cleared.
  FuelStatus s = fc.update(t, 0, 20.0f);
  CHECK_NEAR(s.fuelConsumptionGalMin, 0.0f, 1e-6);
  CHECK(s.fuelConsumptionGalMin >= 0.0f);
}

TEST(FuelComputer, reset_clears_window_and_laps) {
  FuelComputer fc(K);
  runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES, PULSES_PER_SAMPLE_AT_1GPM, 20.0f);
  CHECK(fc.recordLap(90000));
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);

  fc.reset();
  CHECK_EQ(fc.sampleCount(), (uint8_t)0);
  CHECK_EQ(fc.lapCount(), (uint8_t)0);
  // The caller re-offers the same lap time from the bus on the next tick; it
  // must not be counted again into the fresh average.
  CHECK(!fc.recordLap(90000));
  CHECK_EQ(fc.lapCount(), (uint8_t)0);
  FuelStatus s = fc.update(100000, 0, 20.0f);
  CHECK_NEAR(s.fuelConsumptionGalMin, 0.0f, 1e-6);
}

TEST(FuelComputer, endurance_is_remaining_over_rate) {
  FuelComputer fc(K);
  // 1 gal/min, 20 gal tank.
  uint32_t used = 0;
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f, 0, &used);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-4);
  CHECK_NEAR(s.fuelRemainingMins, s.fuelRemainingGals / 1.0f, 1e-3);
}

// With almost no flow the endurance figure tends to infinity; unclamped it
// overflowed the uint16 seconds field on the bus.
TEST(FuelComputer, endurance_is_clamped) {
  FuelComputer fc(K);
  // A trickle: enough to register as flow, far too little to be meaningful.
  // Unclamped this is thousands of minutes and overflows the uint16 seconds
  // field on the bus.
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES, 2, 20.0f);
  CHECK(s.fuelConsumptionGalMin > 0.0f);
  CHECK_NEAR(s.fuelRemainingMins, (float)FuelComputer::MAX_REMAINING_MINS, 0.01);
}

TEST(FuelComputer, zero_flow_yields_zero_endurance_not_infinity) {
  FuelComputer fc(K);
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES, 0, 20.0f);
  CHECK_NEAR(s.fuelConsumptionGalMin, 0.0f, 1e-6);
  CHECK_NEAR(s.fuelRemainingMins, 0.0f, 1e-6);
  CHECK_EQ(s.lapsRemaining, (int16_t)0);
}

TEST(FuelComputer, lap_times_are_deduplicated) {
  FuelComputer fc(K);
  CHECK(fc.recordLap(90000));
  CHECK(!fc.recordLap(90000));  // same value reported again on the bus
  CHECK_EQ(fc.lapCount(), (uint8_t)1);
  CHECK(fc.recordLap(91000));
  CHECK_EQ(fc.lapCount(), (uint8_t)2);
}

TEST(FuelComputer, implausible_lap_times_are_rejected) {
  FuelComputer fc(K);
  CHECK(!fc.recordLap(0));
  CHECK(!fc.recordLap(1));                        // 1 ms
  CHECK(!fc.recordLap(MIN_PLAUSIBLE_LAP_MS - 1));
  CHECK(!fc.recordLap(MAX_PLAUSIBLE_LAP_MS + 1));
  CHECK(!fc.recordLap(0xFFFFFFFFUL));             // all-ones from a bad frame
  CHECK_EQ(fc.lapCount(), (uint8_t)0);
  CHECK(fc.recordLap(MIN_PLAUSIBLE_LAP_MS));
  CHECK_EQ(fc.lapCount(), (uint8_t)1);
}

TEST(FuelComputer, laps_remaining_needs_a_full_lap_history) {
  FuelComputer fc(K);
  fc.recordLap(60000);
  fc.recordLap(61000);
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f);
  CHECK_EQ(s.lapsRemaining, (int16_t)0);  // only two laps known
}

TEST(FuelComputer, computes_laps_remaining_and_gallons_per_lap) {
  FuelComputer fc(K);
  fc.recordLap(60000);  // 1 minute laps
  fc.recordLap(60000 + 1);
  fc.recordLap(60000 - 1);
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f);
  // 1 gal/min over 1 minute laps == 1 gal per lap.
  CHECK_NEAR(s.fuelConsumptionGalLap, 1.0f, 1e-3);
  // Remaining minutes at 1 gal/min over 1 min laps == that many laps.
  CHECK_NEAR((float)s.lapsRemaining, s.fuelRemainingMins, 1.0f);
}

TEST(FuelComputer, laps_remaining_is_clamped) {
  FuelComputer fc(K);
  fc.recordLap(MIN_PLAUSIBLE_LAP_MS);
  fc.recordLap(MIN_PLAUSIBLE_LAP_MS + 1);
  fc.recordLap(MIN_PLAUSIBLE_LAP_MS + 2);
  // A big tank, short laps and a trickle of flow: tens of thousands of laps
  // unclamped, which would wrap the uint16 on the bus.
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES, 2, 200.0f);
  CHECK_EQ(s.lapsRemaining, (int16_t)FuelComputer::MAX_LAPS_REMAINING);
}

TEST(FuelComputer, only_samples_at_the_configured_interval) {
  FuelComputer fc(K);
  fc.update(0, 0, 20.0f);
  CHECK_EQ(fc.sampleCount(), (uint8_t)1);
  // Called far more often than the sample interval, as the main loop does.
  for (uint32_t t = 1; t < FuelComputer::SAMPLE_INTERVAL_MS; t += 10) {
    fc.update(t, 100, 20.0f);
  }
  CHECK_EQ(fc.sampleCount(), (uint8_t)1);
  fc.update(FuelComputer::SAMPLE_INTERVAL_MS, 100, 20.0f);
  CHECK_EQ(fc.sampleCount(), (uint8_t)2);
}

// The firmware calls update() every 500 ms, not once per sample interval, so
// the window length depends on the caller. Pin the real figure: missing this
// is how the averaging period silently changed during the refactor.
TEST(FuelComputer, window_length_at_the_firmware_call_rate) {
  const uint16_t STATUS_PERIOD_MS = 500;
  FuelComputer fc(K);
  uint32_t pulses = 0;
  uint32_t t = 0;
  // Run well past a full window so it is definitely saturated.
  for (uint32_t i = 0; i < 1000; i++) {
    fc.update(t, pulses, 20.0f);
    // 1 gal/min expressed at the 500 ms call rate.
    pulses += PULSES_PER_SAMPLE_AT_1GPM / 2;
    t += STATUS_PERIOD_MS;
  }
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);

  // Samples land 1000 ms apart, so the span is 59 x 1000 ms.
  FuelStatus s = fc.update(t, pulses, 20.0f);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-3);
  CHECK_EQ(FuelComputer::NOMINAL_WINDOW_MS, 59000UL);
}

// Calling faster than the interval must not shorten the window.
TEST(FuelComputer, window_is_unaffected_by_extra_calls) {
  FuelComputer fc(K);
  uint32_t pulses = 0;
  for (uint32_t t = 0; t <= 120000UL; t += 100) {
    fc.update(t, pulses, 20.0f);
    pulses += PULSES_PER_SAMPLE_AT_1GPM / 10;
  }
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);
  FuelStatus s = fc.update(120000UL, pulses, 20.0f);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-2);
}

TEST(FuelComputer, sampling_survives_millis_rollover) {
  FuelComputer fc(K);
  const uint32_t start =
      0xFFFFFFFFUL - (uint32_t)(FuelComputer::HISTORY_SAMPLES / 2) *
                         FuelComputer::SAMPLE_INTERVAL_MS;
  FuelStatus s = runSteadyFlow(fc, FuelComputer::HISTORY_SAMPLES,
                               PULSES_PER_SAMPLE_AT_1GPM, 20.0f, start);
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-3);
}

// The rolling window must stay inside its array no matter how long it runs.
TEST(FuelComputer, long_run_stays_stable) {
  FuelComputer fc(K);
  uint32_t pulses = 0;
  uint32_t t = 0;
  FuelStatus s;
  for (uint32_t i = 0; i < 5000; i++) {
    s = fc.update(t, pulses, 20.0f);
    pulses += PULSES_PER_SAMPLE_AT_1GPM;
    t += FuelComputer::SAMPLE_INTERVAL_MS;
  }
  CHECK_EQ(fc.sampleCount(), FuelComputer::HISTORY_SAMPLES);
  CHECK_NEAR(s.fuelConsumptionGalMin, 1.0f, 1e-3);
  CHECK(s.fuelRemainingGals >= 0.0f);
}
