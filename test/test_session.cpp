// End-to-end simulation of a race stint, driving the same pieces the main loop
// drives and in the same order. This is where the reported symptom - the fuel
// used resetting partway through a session - is pinned down.
#include "test_framework.h"
#include "../src/AutoReset.h"
#include "../src/CanCodec.h"
#include "../src/FuelComputer.h"

using namespace fuel;

namespace {

const uint32_t K = 60000;                 // pulses per gallon
const uint16_t STATUS_PERIOD_MS = 500;    // main loop status cadence
const uint32_t PULSES_PER_SEC_AT_1GPM = 1000;

// Mirrors the arrangement in fuel-usage.ino.
struct Device {
  FuelComputer computer;
  AutoReset refuelDetector;
  Parameters params;
  uint32_t pulses;
  uint32_t resets;
  FuelStatus status;
  // Mirrors the .ino: a reset asked for over the bus is only acted on when
  // this is set, and ignored frames are counted instead.
  bool canResetEnabled;
  uint32_t ignoredCanResetFrames;
  uint8_t lastResetReason;

  Device()
    : computer(K), pulses(0), resets(0), canResetEnabled(true),
      ignoredCanResetFrames(0), lastResetReason(RESET_NONE) {}

  // The commanded-reset branch of loop(), without the 1 s debounce.
  void applyBusReset(bool requested) {
    if (!requested) return;
    if (canResetEnabled) {
      doReset(RESET_COMMANDED);
    } else {
      ignoredCanResetFrames++;
    }
  }

  // The console 'reset' command, which the bus setting must never gate.
  void consoleReset() { doReset(RESET_CONSOLE); }

  void doReset(uint8_t reason) {
    pulses = 0;
    computer.reset();
    refuelDetector.disarm();
    resets++;
    lastResetReason = reason;
  }

  // One pass of the 500 ms status branch of loop().
  void tick(uint32_t nowMs) {
    computer.recordLap(params.lastLapMs);
    status = computer.update(nowMs, pulses, params.capacityGals);

    if (params.autoReset) {
      if (refuelDetector.update(nowMs, params.fuelFull, status.fuelUsedGals,
                                params.speedMph)) {
        // resetFuel() in the firmware disarms; rearm() here would let the
        // harness pass while production diverged.
        doReset(RESET_REFUEL);
        status = computer.update(nowMs, pulses, params.capacityGals);
      }
    } else {
      refuelDetector.rearm();
    }
  }

  void burnFor(uint32_t ms, float galPerMin) {
    pulses += (uint32_t)((float)PULSES_PER_SEC_AT_1GPM * galPerMin *
                         ((float)ms / 1000.0f));
  }
};

// Configure the device over the bus the way the dash would.
void sendSetup(Device& d, float capacityGals, uint32_t lapMs, bool autoReset) {
  uint8_t frame[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  writeU16(&frame[0], (uint16_t)(capacityGals * 100.0f));
  writeU32(&frame[2], lapMs);
  frame[6] = autoReset ? 1 : 0;
  decodeFrame(CAN_ID_RX_CAPACITY, frame, 8, false, d.params);
}

bool sendState(Device& d, float speedMph, float levelGals, bool full,
               bool reset, uint8_t len = 8) {
  uint8_t frame[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  writeU16(&frame[0], (uint16_t)(speedMph * 10.0f));
  writeU16(&frame[2], (uint16_t)(levelGals * 100.0f));
  frame[4] = full ? 1 : 0;
  frame[5] = reset ? 1 : 0;
  return decodeFrame(CAN_ID_RX_STATE, frame, len, false, d.params).resetRequested;
}

}  // namespace

// A 30 minute stint at racing pace, with the tank-full float switch chattering
// the way it does under braking and over kerbs. The counter must survive it.
TEST(Session, no_spurious_reset_during_a_long_stint) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);

  uint32_t lap = 0;
  for (uint32_t t = 0; t < 1800000UL; t += STATUS_PERIOD_MS) {
    // Float switch reads full for a single 500 ms sample now and then.
    bool sloshing = ((t / STATUS_PERIOD_MS) % 37) == 0;
    sendState(d, 85.0f, 12.0f, sloshing, false);

    d.burnFor(STATUS_PERIOD_MS, 0.6f);
    if (t > 0 && (t % 90000UL) == 0) {
      lap++;
      d.params.lastLapMs = 90000UL + lap;  // each lap slightly different
    }
    d.tick(t);
  }

  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 15.0f);
  CHECK(d.status.fuelConsumptionGalMin > 0.0f);
}

// Even while stationary under a full course caution, a tank that reads full
// must not reset the counter as long as the car has not actually been refueled
// - which is what the fuel-used gate is for.
TEST(Session, no_reset_while_stopped_with_a_flickering_sensor) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);

  // Burn most of a tank first, so the fuel-used gate is satisfied.
  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 80.0f, 12.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 4.0f);

  // Now sit stationary for two minutes with fuel sloshing onto the switch.
  uint32_t base = 600000UL;
  for (uint32_t t = 0; t < 120000UL; t += STATUS_PERIOD_MS) {
    bool sloshing = ((t / STATUS_PERIOD_MS) % 4) < 2;  // 1 s on, 1 s off
    sendState(d, 0.0f, 12.0f, sloshing, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 0u);
}

// A genuine pit stop: stopped, tank filled, switch held for longer than the
// dwell. Exactly one reset, and it must not repeat while still sitting there.
TEST(Session, genuine_refuel_resets_exactly_once) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);

  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 80.0f, 12.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK(d.status.fuelUsedGals > 9.0f);

  // Pit stop: stationary, tank full, held for a minute.
  uint32_t base = 600000UL;
  for (uint32_t t = 0; t < 60000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 0.0f, 20.0f, true, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 1u);
  CHECK_NEAR(d.status.fuelUsedGals, 0.0f, 0.01);

  // Still sitting in the pits with a full tank: no further resets.
  base += 60000UL;
  for (uint32_t t = 0; t < 120000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 0.0f, 20.0f, true, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 1u);
}

// The dwell must not be satisfied by a stop that is shorter than it.
TEST(Session, splash_stop_shorter_than_the_dwell_does_not_reset) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);
  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 80.0f, 12.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }

  uint32_t base = 600000UL;
  for (uint32_t t = 0; t < 8000UL; t += STATUS_PERIOD_MS) {  // 8 s, dwell is 10
    sendState(d, 0.0f, 20.0f, true, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 0u);
}

// A tank-full line stuck asserted - a lost sender ground is enough, since any
// non-zero byte reads as full - plus the brief stops a race session naturally
// contains: pit entry, the grid, a spin, a full course yellow. None of them
// may zero the counter, because the car was never actually refueled.
TEST(Session, stuck_full_sender_survives_a_session_of_brief_stops) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);
  // Arm it first, so this tests the continuous speed gating rather than
  // stopping at the arming guard.
  d.refuelDetector.setArmDwellMs(0);
  sendState(d, 85.0f, 12.0f, false, false);
  d.tick(0);

  for (uint32_t t = 0; t < 1800000UL; t += STATUS_PERIOD_MS) {
    // Stationary for a single 500 ms sample every two minutes.
    bool stopped = (t % 120000UL) == 0 && t > 0;
    sendState(d, stopped ? 0.0f : 85.0f, 12.0f, true /* stuck full */, false);
    if (!stopped) d.burnFor(STATUS_PERIOD_MS, 0.6f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 15.0f);
}

// Rolling slowly through the pit lane below the speed gate, without stopping
// long enough to refuel, must not count either.
TEST(Session, slow_lap_through_the_pits_does_not_reset) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);
  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 80.0f, 12.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }

  // 30 s crawling: mostly under 3 mph but never stationary for a full 10 s.
  uint32_t base = 600000UL;
  for (uint32_t t = 0; t < 30000UL; t += STATUS_PERIOD_MS) {
    float speed = ((t / STATUS_PERIOD_MS) % 16) < 12 ? 1.0f : 8.0f;
    sendState(d, speed, 20.0f, true, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 0u);
}

// The failure seen on the car. The tank-full line is a level indication that
// reads asserted over a wide range of tank contents, so it was high on every
// frame of a real capture. Burn most of a tank, pit in, and sit there: the
// counter must survive, because nothing was actually refueled.
TEST(Session, always_high_tank_full_line_survives_a_long_pit_stop) {
  Device d;
  sendSetup(d, 18.8f, 90000, true);

  // A stint with the line asserted throughout, as observed.
  for (uint32_t t = 0; t < 900000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 85.0f, 0.0f, true, false);
    d.burnFor(STATUS_PERIOD_MS, 0.6f);
    d.tick(t);
  }
  CHECK(d.status.fuelUsedGals > 4.0f);
  CHECK_EQ(d.resets, 0u);

  // Pull in and stop for five minutes, exactly where it used to zero itself.
  uint32_t base = 900000UL;
  for (uint32_t t = 0; t < 300000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 0.0f, 0.0f, true, false);
    d.tick(base + t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 4.0f);
}

// ...but a tank that actually runs down and is then filled still works.
TEST(Session, refuel_after_the_line_drops_is_still_detected) {
  Device d;
  sendSetup(d, 18.8f, 90000, true);

  uint32_t t = 0;
  for (; t < 600000UL; t += STATUS_PERIOD_MS) {   // full, on track
    sendState(d, 85.0f, 0.0f, true, false);
    d.burnFor(STATUS_PERIOD_MS, 0.8f);
    d.tick(t);
  }
  for (; t < 1200000UL; t += STATUS_PERIOD_MS) {  // level below the switch
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 0.8f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);

  for (; t < 1230000UL; t += STATUS_PERIOD_MS) {  // stopped, not yet fueled
    sendState(d, 0.0f, 0.0f, false, false);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);

  for (; t < 1290000UL; t += STATUS_PERIOD_MS) {  // fuel goes in
    sendState(d, 0.0f, 0.0f, true, false);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 1u);
  CHECK_NEAR(d.status.fuelUsedGals, 0.0f, 0.01);
}

// A splash-and-go after a couple of laps: the tank-full line does transition,
// and the car is stopped, but too little has been burned for this to be a
// stint refuel.
TEST(Session, refuel_after_only_a_little_fuel_burned_does_not_reset) {
  Device d;
  sendSetup(d, 18.8f, 90000, true);

  uint32_t t = 0;
  for (; t < 180000UL; t += STATUS_PERIOD_MS) {   // ~2 laps, line low
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 0.6f);
    d.tick(t);
  }
  CHECK(d.status.fuelUsedGals < 4.0f);

  for (; t < 300000UL; t += STATUS_PERIOD_MS) {   // stopped, line asserted
    sendState(d, 0.0f, 0.0f, true, false);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 1.0f);
}

// Auto-reset switched off means the tank-full signal is ignored entirely.
TEST(Session, auto_reset_disabled_never_resets) {
  Device d;
  sendSetup(d, 20.0f, 90000, false);
  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 0.0f, 20.0f, true, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelUsedGals > 9.0f);
}

// A frame too short to carry the reset byte must never be read as a reset.
TEST(Session, short_state_frames_do_not_trigger_a_reset) {
  Device d;
  sendSetup(d, 20.0f, 90000, false);
  for (uint8_t len = 0; len < 6; len++) {
    CHECK(!sendState(d, 40.0f, 12.0f, false, true, len));
  }
  CHECK(sendState(d, 40.0f, 12.0f, false, true, 6));
}

// A full stint reported in metric should describe the same physical situation.
TEST(Session, metric_mode_tracks_the_same_stint) {
  Device d;
  // Capacity arrives as liters: 75.7 L is 20 gal.
  uint8_t frame[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  writeU16(&frame[0], (uint16_t)(75.708f * 100.0f));
  writeU32(&frame[2], 90000);
  frame[6] = 1;
  decodeFrame(CAN_ID_RX_CAPACITY, frame, 8, true, d.params);
  CHECK_NEAR(d.params.capacityGals, 20.0f, 0.01);

  for (uint32_t t = 0; t < 600000UL; t += STATUS_PERIOD_MS) {
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);

  TxFrames tx;
  encodeStatus(d.status, true, tx);
  // 10 gal used is about 37.85 L, sent x100.
  CHECK_NEAR((float)readU16(&tx.usage[4]) / 100.0f, 37.85f, 0.2f);
}

// Sanity check that nothing wanders as the timer wraps mid-session.
TEST(Session, survives_millis_rollover_mid_stint) {
  Device d;
  sendSetup(d, 20.0f, 90000, true);
  uint32_t start = 0xFFFFFFFFUL - 300000UL;  // wraps five minutes in
  for (uint32_t i = 0; i < 2400; i++) {      // 20 minutes
    uint32_t t = start + i * STATUS_PERIOD_MS;
    sendState(d, 80.0f, 12.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 0.5f);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 0u);
  CHECK(d.status.fuelConsumptionGalMin > 0.4f);
  CHECK(d.status.fuelConsumptionGalMin < 0.6f);
}

// ---------------------------------------------------------------------------
// Resets commanded over the bus
//
// A live capture caught the dash commanding a reset on a momentary blip of
// the tank-full switch, which wiped the counter mid-session. setcanreset 0
// makes the device ignore the command.
// ---------------------------------------------------------------------------

TEST(Session, a_bus_reset_is_acted_on_when_enabled) {
  Device d;
  d.canResetEnabled = true;
  sendSetup(d, 18.8f, 90000, false);
  for (uint32_t t = 0; t < 300000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK(d.status.fuelUsedGals > 4.0f);

  d.applyBusReset(sendState(d, 0.0f, 0.0f, false, true));
  CHECK_EQ(d.resets, 1u);
  CHECK_EQ(d.lastResetReason, RESET_COMMANDED);
  CHECK_EQ(d.ignoredCanResetFrames, 0u);
}

TEST(Session, a_bus_reset_is_ignored_and_counted_when_disabled) {
  Device d;
  d.canResetEnabled = false;
  sendSetup(d, 18.8f, 90000, false);
  for (uint32_t t = 0; t < 300000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  float before = d.status.fuelUsedGals;
  CHECK(before > 4.0f);

  // A dash holding the byte set adds one per frame.
  for (uint8_t i = 0; i < 5; i++) {
    d.applyBusReset(sendState(d, 0.0f, 0.0f, false, true));
  }
  CHECK_EQ(d.resets, 0u);
  CHECK_EQ(d.ignoredCanResetFrames, 5u);
  d.tick(300000UL);
  CHECK_NEAR(d.status.fuelUsedGals, before, 0.01);
}

// Disabling the bus command must not take away the manual override.
TEST(Session, the_console_reset_still_works_when_the_bus_command_is_off) {
  Device d;
  d.canResetEnabled = false;
  sendSetup(d, 18.8f, 90000, false);
  for (uint32_t t = 0; t < 300000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }
  CHECK(d.status.fuelUsedGals > 4.0f);

  d.consoleReset();
  CHECK_EQ(d.resets, 1u);
  CHECK_EQ(d.lastResetReason, RESET_CONSOLE);
  d.tick(300000UL);
  CHECK_NEAR(d.status.fuelUsedGals, 0.0f, 0.01);
}

// ...and must not take away refuel detection either.
TEST(Session, refuel_detection_still_works_when_the_bus_command_is_off) {
  Device d;
  d.canResetEnabled = false;
  sendSetup(d, 18.8f, 90000, true);

  uint32_t t = 0;
  for (; t < 600000UL; t += STATUS_PERIOD_MS) {   // level below the switch
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 0.8f);
    d.tick(t);
  }
  for (; t < 660000UL; t += STATUS_PERIOD_MS) {   // stopped, tank filled
    sendState(d, 0.0f, 0.0f, true, false);
    d.tick(t);
  }
  CHECK_EQ(d.resets, 1u);
  CHECK_EQ(d.lastResetReason, RESET_REFUEL);
  CHECK_EQ(d.ignoredCanResetFrames, 0u);
}

// The three routes have to stay distinguishable, which is the whole point of
// recording a reason.
TEST(Session, each_reset_route_records_its_own_reason) {
  Device d;
  sendSetup(d, 18.8f, 90000, false);
  for (uint32_t t = 0; t < 300000UL; t += STATUS_PERIOD_MS) {
    sendState(d, 85.0f, 0.0f, false, false);
    d.burnFor(STATUS_PERIOD_MS, 1.0f);
    d.tick(t);
  }

  d.applyBusReset(sendState(d, 0.0f, 0.0f, false, true));
  CHECK_EQ(d.lastResetReason, RESET_COMMANDED);
  d.consoleReset();
  CHECK_EQ(d.lastResetReason, RESET_CONSOLE);
  CHECK_EQ(d.resets, 2u);
}
