#include "test_framework.h"
#include "../src/AutoReset.h"

using namespace fuel;

namespace {
// A refuel-shaped situation: plenty burned, car stationary.
const float USED = 10.0f;
const float STOPPED = 0.0f;

// The detector only acts once the tank-full line has been held low for the
// arming period. Most of these tests are about the dwell and the gates rather
// than about arming timing, so they switch the arming period off and arm with
// a single low sample. The tests that are about arming drive it properly.
void arm(AutoReset& ar, uint32_t atMs) {
  ar.setArmDwellMs(0);
  ar.update(atMs, false, USED, STOPPED);
}

// Arm the way a real tank does: hold the line low for the full period.
uint32_t armByRunningDown(AutoReset& ar, uint32_t fromMs) {
  uint32_t until = fromMs + AutoReset::DEFAULT_ARM_DWELL_MS;
  for (uint32_t t = fromMs; t <= until; t += 500) {
    ar.update(t, false, USED, STOPPED);
  }
  return until;
}
}  // namespace

// The regression test for the reported fault. Before the fix the dwell timer
// was cleared on every update where the signal was already held, so the
// elapsed time was measured from timestamp zero - i.e. from boot - and the
// reset fired on the second update, 500 ms in, instead of after 10 s.
TEST(AutoReset, does_not_fire_before_dwell_elapses) {
  AutoReset ar;
  arm(ar, 0);
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
  arm(ar, t0);
  CHECK(!ar.update(t0, true, USED, STOPPED));
  CHECK(!ar.update(t0 + 500, true, USED, STOPPED));
  CHECK(!ar.update(t0 + 9999, true, USED, STOPPED));
  CHECK(ar.update(t0 + 10000, true, USED, STOPPED));
}

TEST(AutoReset, fires_once_dwell_is_satisfied) {
  AutoReset ar;
  arm(ar, 1000);
  CHECK(!ar.update(1000, true, USED, STOPPED));
  CHECK(ar.update(11000, true, USED, STOPPED));
}

TEST(AutoReset, dropout_restarts_the_dwell) {
  AutoReset ar;
  arm(ar, 0);
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
  arm(ar, 0);  // armed, so the used gate is the only thing holding it off
  // Topping up after a single lap is not a pit refuel.
  for (uint32_t t = 0; t <= 30000UL; t += 500) {
    CHECK(!ar.update(t, true, 1.0f, STOPPED));
  }
}

TEST(AutoReset, requires_vehicle_to_be_stopped) {
  AutoReset ar;
  arm(ar, 0);  // armed, so the speed gate is the only thing holding it off
  for (uint32_t t = 0; t <= 30000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, 55.0f));
  }
}

// Moving at any point during the dwell restarts it, so the car has to be
// stationary for the full period rather than merely at the moment it expires.
TEST(AutoReset, moving_during_the_dwell_restarts_it) {
  AutoReset ar;
  arm(ar, 0);
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
  // Armed first, so this exercises the continuous speed gating rather than
  // stopping at the arming guard: the honest model of a line that was low and
  // has since gone stuck high.
  arm(ar, 0);
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
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(5000, true, 1.0f, STOPPED));  // below the threshold
  CHECK(!ar.update(10000, true, USED, STOPPED));
  CHECK(!ar.update(19999, true, USED, STOPPED));
  CHECK(ar.update(20000, true, USED, STOPPED));
}

TEST(AutoReset, disarms_after_firing_so_it_does_not_repeat) {
  AutoReset ar;
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
  // Firing disarmed it, so the tank going on reading full cannot trigger a
  // second reset however long it is held.
  for (uint32_t t = 10500; t <= 120000UL; t += 500) {
    CHECK(!ar.update(t, true, 0.0f, STOPPED));
  }
}

// If enough fuel really is burned again, a fresh dwell must be served in full
// before it can fire a second time.
TEST(AutoReset, a_second_refuel_needs_a_complete_fresh_dwell) {
  AutoReset ar;
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
  CHECK(!ar.update(10500, true, 0.0f, STOPPED));   // counter zeroed
  // Firing disarmed it, so the line has to fall and rise again before a
  // second refuel can be recognized, on top of a fresh dwell.
  CHECK(!ar.update(20000, true, USED, STOPPED));
  arm(ar, 25000);
  CHECK(!ar.update(30000, true, USED, STOPPED));   // burned another stint
  CHECK(!ar.update(39999, true, USED, STOPPED));
  CHECK(ar.update(40000, true, USED, STOPPED));
}

// millis() wraps every ~49.7 days. Unsigned subtraction makes the dwell
// measurement survive it; a signed or naive comparison would not.
TEST(AutoReset, dwell_survives_millis_rollover) {
  AutoReset ar;
  const uint32_t nearMax = 4294960000UL;  // ~7.3 s before wrap
  arm(ar, nearMax);
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
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(ar.isDwelling());
  CHECK_EQ(ar.dwellElapsedMs(0), 0u);
  CHECK_EQ(ar.dwellElapsedMs(4000), 4000u);
  CHECK(!ar.update(9999, true, USED, STOPPED));
  CHECK(ar.update(10000, true, USED, STOPPED));
}

TEST(AutoReset, rearm_clears_pending_dwell) {
  AutoReset ar;
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  ar.rearm();  // drops the dwell, keeps the arming
  CHECK(!ar.isDwelling());
  CHECK(!ar.update(10000, true, USED, STOPPED));  // clock restarted at 10000
  CHECK(ar.update(20000, true, USED, STOPPED));
}

// A NaN reading must never satisfy a gate. This works because any comparison
// against NaN is false, so it is the gates themselves being verified rather
// than a separate guard.
TEST(AutoReset, nan_inputs_never_fire) {
  const float nan = 0.0f / 0.0f;

  AutoReset used;
  arm(used, 0);
  for (uint32_t t = 0; t <= 60000UL; t += 500) {
    CHECK(!used.update(t, true, nan, STOPPED));
  }

  AutoReset speed;
  arm(speed, 0);
  for (uint32_t t = 0; t <= 60000UL; t += 500) {
    CHECK(!speed.update(t, true, USED, nan));
  }
}

TEST(AutoReset, dwell_is_configurable) {
  AutoReset ar;
  ar.setDwellMs(30000);
  arm(ar, 0);
  CHECK(!ar.update(0, true, USED, STOPPED));
  CHECK(!ar.update(29999, true, USED, STOPPED));
  CHECK(ar.update(30000, true, USED, STOPPED));
}


// ---------------------------------------------------------------------------
// Arming: the tank-full line must be seen low before it can mean "refueled"
// ---------------------------------------------------------------------------

// The fault seen on the car. The tank-full line is a level indication, not a
// refuel event, and it reads asserted across a wide range of tank contents -
// it was high on all 516 frames captured in one session. Treating "high" as
// "just refueled" wiped the counter at every pit stop.
TEST(AutoReset, a_line_that_has_always_been_high_never_fires) {
  AutoReset ar;
  CHECK(!ar.isArmed());
  // Half an hour stationary, plenty of fuel burned, signal asserted throughout.
  for (uint32_t t = 0; t <= 1800000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, STOPPED));
  }
  CHECK(!ar.isArmed());
}

// Driving a whole session with the line high, then stopping in the pits, is
// the exact sequence that was zeroing the counter.
TEST(AutoReset, stop_after_a_session_with_the_line_always_high_does_not_fire) {
  AutoReset ar;
  for (uint32_t t = 0; t < 600000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, 85.0f));  // on track
  }
  for (uint32_t t = 600000UL; t < 900000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, STOPPED));  // five minutes in the pits
  }
}

// A single low sample must not arm. A float switch that dips low once under
// braking on a full tank is as physical as one that dips high, and an
// undebounced latch would let one such sample in a whole session re-create the
// fault this exists to prevent.
TEST(AutoReset, one_low_sample_does_not_arm) {
  AutoReset ar;
  CHECK(!ar.update(0, false, USED, STOPPED));
  CHECK(!ar.isArmed());
  CHECK(ar.isArming());
  // Back high: the accumulated arming is discarded.
  CHECK(!ar.update(500, true, USED, STOPPED));
  CHECK(!ar.isArmed());
  CHECK(!ar.isArming());
  // ...and a stop with the line high still cannot fire.
  for (uint32_t t = 1000; t <= 60000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, STOPPED));
  }
}

// The whole H1 scenario end to end: a session at speed, one glitched low
// sample, then a long pit stop.
TEST(AutoReset, one_glitched_low_sample_then_a_pit_stop_does_not_fire) {
  AutoReset ar;
  uint32_t t = 0;
  for (; t < 600000UL; t += 500) CHECK(!ar.update(t, true, USED, 85.0f));
  CHECK(!ar.update(t, false, USED, 85.0f));  // single glitch
  t += 500;
  for (; t < 700000UL; t += 500) CHECK(!ar.update(t, true, USED, 85.0f));
  for (; t < 760000UL; t += 500) CHECK(!ar.update(t, true, USED, STOPPED));
  CHECK(!ar.isArmed());
}

TEST(AutoReset, holding_the_line_low_for_the_arming_period_arms_it) {
  AutoReset ar;
  for (uint32_t t = 0; t < AutoReset::DEFAULT_ARM_DWELL_MS; t += 500) {
    ar.update(t, false, USED, STOPPED);
    CHECK(!ar.isArmed());
  }
  ar.update(AutoReset::DEFAULT_ARM_DWELL_MS, false, USED, STOPPED);
  CHECK(ar.isArmed());
}

// The line coming back high part-way through discards the progress, exactly
// as an interrupted dwell does.
TEST(AutoReset, interrupting_the_low_period_restarts_arming) {
  AutoReset ar;
  for (uint32_t t = 0; t < 8000UL; t += 500) ar.update(t, false, USED, STOPPED);
  CHECK(!ar.isArmed());
  ar.update(8000, true, USED, STOPPED);        // momentarily high
  CHECK(!ar.isArming());
  for (uint32_t t = 8500; t < 16000UL; t += 500) {
    ar.update(t, false, USED, STOPPED);
  }
  CHECK(!ar.isArmed());                        // 7.5 s, not yet enough
  armByRunningDown(ar, 16000);
  CHECK(ar.isArmed());
}

TEST(AutoReset, arming_period_is_configurable) {
  AutoReset ar;
  ar.setArmDwellMs(3000);
  ar.update(0, false, USED, STOPPED);
  CHECK(!ar.isArmed());
  ar.update(2999, false, USED, STOPPED);
  CHECK(!ar.isArmed());
  ar.update(3000, false, USED, STOPPED);
  CHECK(ar.isArmed());
}

TEST(AutoReset, arming_survives_millis_rollover) {
  AutoReset ar;
  const uint32_t start = 0xFFFFFFFFUL - 4000UL;
  for (uint32_t i = 0; i <= AutoReset::DEFAULT_ARM_DWELL_MS; i += 500) {
    ar.update((uint32_t)(start + i), false, USED, STOPPED);
  }
  CHECK(ar.isArmed());
}

// A real refuel: tank runs down so the switch releases, car pits, tank is
// filled, switch asserts again.
TEST(AutoReset, a_genuine_refuel_sequence_fires_once) {
  AutoReset ar;
  uint32_t t = 0;
  // On track with a full tank - cannot fire, never been low.
  for (; t < 300000UL; t += 500) CHECK(!ar.update(t, true, USED, 85.0f));
  // Level falls below the switch and stays there, which is what arms it.
  for (; t < 600000UL; t += 500) CHECK(!ar.update(t, false, USED, 85.0f));
  CHECK(ar.isArmed());
  // Pit entry, still not refueled.
  for (; t < 610000UL; t += 500) CHECK(!ar.update(t, false, USED, STOPPED));
  // Fuel goes in; the switch asserts and is held.
  uint32_t fills = 0;
  for (; t < 660000UL; t += 500) {
    if (ar.update(t, true, USED, STOPPED)) fills++;
  }
  CHECK_EQ(fills, 1u);
  CHECK(!ar.isArmed());
}

// Sloshing onto the switch while stopped still must not count, even once
// armed, because the dwell has to be served.
TEST(AutoReset, armed_but_sloshing_does_not_fire) {
  AutoReset ar;
  arm(ar, 0);
  for (uint32_t t = 500; t < 120000UL; t += 500) {
    bool sloshing = ((t / 500) % 4) < 2;  // 1 s on, 1 s off
    CHECK(!ar.update(t, sloshing, USED, STOPPED));
  }
}

TEST(AutoReset, disarm_requires_a_fresh_transition) {
  AutoReset ar;
  arm(ar, 0);
  CHECK(ar.isArmed());
  ar.disarm();
  CHECK(!ar.isArmed());
  for (uint32_t t = 0; t <= 60000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, STOPPED));
  }
  arm(ar, 60500);
  CHECK(!ar.update(61000, true, USED, STOPPED));
  CHECK(ar.update(71000, true, USED, STOPPED));
}

// rearm() drops the dwell but keeps the arming, so a momentary gap in
// evaluation - auto-reset toggled, or the bus going quiet - does not throw
// away a transition that genuinely happened.
// disarm() has to drop a low period that is still accumulating, not just the
// arming already earned, or the caller could zero the counter and then have a
// stale part-served period complete moments later.
TEST(AutoReset, disarm_drops_a_low_period_in_progress) {
  AutoReset ar;
  CHECK(!ar.update(0, false, USED, STOPPED));
  CHECK(ar.isArming());
  CHECK(!ar.isArmed());
  ar.disarm();
  CHECK(!ar.isArming());
  CHECK(!ar.isArmed());
  // Resuming the low signal starts a fresh period rather than completing the
  // old one.
  CHECK(!ar.update(AutoReset::DEFAULT_ARM_DWELL_MS, false, USED, STOPPED));
  CHECK(!ar.isArmed());
}

TEST(AutoReset, rearm_keeps_earned_arming_while_disarm_does_not) {
  AutoReset ar;
  arm(ar, 0);
  ar.rearm();
  CHECK(ar.isArmed());
  ar.disarm();
  CHECK(!ar.isArmed());
  CHECK(!ar.isArming());
}

// The detector is not evaluated while auto-reset is off or the state frames
// are stale; the main loop calls rearm() throughout. A low period that was
// only partly served when that gap began has not been earned, so it must not
// be resumed against a timestamp from before the gap - otherwise two low
// samples an arbitrary time apart would arm the detector, which is the
// single-sample latch the debounce exists to prevent.
TEST(AutoReset, a_part_served_low_period_does_not_survive_a_gap) {
  AutoReset ar;
  CHECK(!ar.update(0, false, USED, STOPPED));   // one low sample
  CHECK(ar.isArming());

  // 60 s where the detector is not evaluated at all.
  for (uint32_t t = 500; t <= 60000UL; t += 500) ar.rearm();
  CHECK(!ar.isArming());
  CHECK(!ar.isArmed());

  // A second low sample long afterwards must start a fresh period, not
  // complete the old one.
  CHECK(!ar.update(60500, false, USED, STOPPED));
  CHECK(!ar.isArmed());
  for (uint32_t t = 61000; t <= 120000UL; t += 500) {
    CHECK(!ar.update(t, true, USED, STOPPED));
  }
}

// Arming already earned does survive such a gap, because a real refuel did
// happen and the remaining gates still apply.
TEST(AutoReset, earned_arming_survives_a_gap_in_evaluation) {
  AutoReset ar;
  armByRunningDown(ar, 0);
  CHECK(ar.isArmed());
  for (uint32_t t = 0; t < 60000UL; t += 500) ar.rearm();
  CHECK(ar.isArmed());
  CHECK(!ar.update(70000, true, USED, STOPPED));
  CHECK(ar.update(80000, true, USED, STOPPED));
}
