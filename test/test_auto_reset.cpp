#include "test_framework.h"
#include "../src/AutoReset.h"

using namespace fuel;

namespace {
// A refuel-shaped situation: plenty burned, car stationary.
const float USED = 10.0f;
const float STOPPED = 0.0f;
}  // namespace

// The regression test for the reported fault. Before the fix the dwell timer
// was cleared on every update where the signal was already held, so the
// elapsed time was measured from timestamp zero - i.e. from boot - and the
// reset fired on the second update, 500 ms in, instead of after 10 s.
TEST(AutoReset, does_not_fire_before_dwell_elapses) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(500, true, USED, STOPPED));
  CHECK(!ar.update(1000, true, USED, STOPPED));
  CHECK(!ar.update(5000, true, USED, STOPPED));
  CHECK(!ar.update(9999, true, USED, STOPPED));
}

// Same sequence but starting well after boot, which is when the old code was
// at its worst: millis() was already far past the threshold.
TEST(AutoReset, does_not_fire_early_when_run_starts_late) {
  AutoReset ar;
  const uint32_t t0 = 3600000UL;  // one hour in
  CHECK(!ar.update(t0, true, USED, STOPPED));
  CHECK(!ar.update(t0 + 500, true, USED, STOPPED));
  CHECK(!ar.update(t0 + 9999, true, USED, STOPPED));
  CHECK(ar.update(t0 + 10000, true, USED, STOPPED));
}

TEST(AutoReset, fires_once_dwell_is_satisfied) {
  AutoReset ar;
  CHECK(!ar.update(1000, true, USED, STOPPED));
  CHECK(ar.update(11000, true, USED, STOPPED));
}

TEST(AutoReset, dropout_restarts_the_dwell) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(9000, true, USED, STOPPED));
  // Sensor flickers low - fuel sloshing off the float switch.
  CHECK(!ar.update(9500, false, USED, STOPPED));
  // The dwell restarts from the update where the signal came back, so the
  // original 10 s deadline passes without firing.
  CHECK(!ar.update(10000, true, USED, STOPPED));
  CHECK(!ar.update(19999, true, USED, STOPPED));
  CHECK(ar.update(20000, true, USED, STOPPED));
}

TEST(AutoReset, intermittent_signal_never_fires) {
  AutoReset ar;
  // A float switch chattering once a second can never accumulate the dwell.
  for (uint32_t t = 0; t < 120000UL; t += 500) {
    bool full = ((t / 500) % 2) == 0;
    CHECK(!ar.update(t, full, USED, STOPPED));
  }
}

TEST(AutoReset, requires_meaningful_fuel_burned) {
  AutoReset ar;
  // Topping up after a single lap is not a pit refuel.
  for (uint32_t t = 0; t <= 30000UL; t += 500) {
    CHECK(!ar.update(t, true, 1.0f, STOPPED));
  }
}

TEST(AutoReset, requires_vehicle_to_be_stopped) {
  AutoReset ar;
  for (uint32_t t = 0; t <= 30000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, 55.0f));
  }
}

// Moving at any point during the dwell restarts it, so the car has to be
// stationary for the full period rather than merely at the moment it expires.
TEST(AutoReset, moving_during_the_dwell_restarts_it) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(9000, true, USED, 40.0f));   // rolls forward in the pit lane
  CHECK(!ar.update(10000, true, USED, STOPPED));  // clock restarts here
  CHECK(!ar.update(19999, true, USED, STOPPED));
  CHECK(ar.update(20000, true, USED, STOPPED));
}

// The scenario this guards against: a tank-full line stuck asserted for a
// whole stint plus one momentary stop. Without continuous gating the dwell
// would already have expired and that single slow sample would reset.
TEST(AutoReset, stuck_full_sender_plus_one_brief_stop_does_not_reset) {
  AutoReset ar;
  for (uint32_t t = 0; t < 45000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, 85.0f));  // full the whole time, at speed
  }
  CHECK(!ar.update(45000, true, USED, 1.0f));   // one sample below the limit
  CHECK(!ar.update(45500, true, USED, 85.0f));  // and back up to speed
  for (uint32_t t = 46000; t < 90000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, 85.0f));
  }
}

// Fuel used dropping below the threshold mid-dwell also restarts it.
TEST(AutoReset, used_gate_must_hold_for_the_whole_dwell) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(5000, true, 1.0f, STOPPED));  // below the threshold
  CHECK(!ar.update(10000, true, USED, STOPPED));
  CHECK(!ar.update(19999, true, USED, STOPPED));
  CHECK(ar.update(20000, true, USED, STOPPED));
}

TEST(AutoReset, rearms_after_firing_so_it_does_not_repeat) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
  // Caller has now zeroed the counter, so the used gate holds it off no matter
  // how long the tank goes on reading full.
  for (uint32_t t = 10500; t <= 120000UL; t += 500) {
    CHECK(!ar.update(t, true, 0.0f, STOPPED));
  }
}

// If enough fuel really is burned again, a fresh dwell must be served in full
// before it can fire a second time.
TEST(AutoReset, a_second_refuel_needs_a_complete_fresh_dwell) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
  CHECK(!ar.update(10500, true, 0.0f, STOPPED));   // counter zeroed
  CHECK(!ar.update(30000, true, USED, STOPPED));   // burned another stint
  CHECK(!ar.update(39999, true, USED, STOPPED));
  CHECK(ar.update(40000, true, USED, STOPPED));
}

// millis() wraps every ~49.7 days. Unsigned subtraction makes the dwell
// measurement survive it; a signed or naive comparison would not.
TEST(AutoReset, dwell_survives_millis_rollover) {
  AutoReset ar;
  const uint32_t nearMax = 4294960000UL;  // ~7.3 s before wrap
  CHECK(!ar.update(nearMax, true, USED, STOPPED));
  CHECK(!ar.update(nearMax + 5000UL, true, USED, STOPPED));  // still pre-wrap
  CHECK(!ar.update((uint32_t)(nearMax + 9999UL), true, USED, STOPPED));
  CHECK(ar.update((uint32_t)(nearMax + 10000UL), true, USED, STOPPED));
}

// millis() returns 0 for the first millisecond after boot and again after each
// rollover. The old zero-timestamp sentinel could not tell "started at 0" from
// "not started".
TEST(AutoReset, treats_timestamp_zero_as_a_real_start_time) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.isDwelling());
  CHECK_EQ(ar.dwellElapsedMs(0), 0u);
  CHECK_EQ(ar.dwellElapsedMs(4000), 4000u);
  CHECK(!ar.update(9999, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
}

TEST(AutoReset, rearm_clears_pending_dwell) {
  AutoReset ar;
  CHECK(!ar.update(0, true, USED, STOPPED));
  ar.rearm();
  CHECK(!ar.isDwelling());
  CHECK(!ar.update(10000, true, USED, STOPPED));  // clock restarted at 10000
  CHECK(ar.update(20000, true, USED, STOPPED));
}

TEST(AutoReset, rejects_nan_inputs) {
  AutoReset ar;
  const float nan = 0.0f / 0.0f;
  CHECK(!ar.update(0, true, nan, STOPPED));
  CHECK(!ar.update(10000, true, nan, STOPPED));
  CHECK(!ar.update(20000, true, USED, nan));
}

TEST(AutoReset, dwell_is_configurable) {
  AutoReset ar;
  ar.setDwellMs(30000);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(29999, true, USED, STOPPED));
  CHECK(ar.update(30000, true, USED, STOPPED));
}
