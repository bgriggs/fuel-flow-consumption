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

  Device() : computer(K), pulses(0), resets(0) {}

  // One pass of the 500 ms status branch of loop().
  void tick(uint32_t nowMs) {
    computer.recordLap(params.lastLapMs);
    status = computer.update(nowMs, pulses, params.capacityGals);

    if (params.autoReset) {
      if (refuelDetector.update(nowMs, params.fuelFull, status.fuelUsedGals,
                                params.speedMph)) {
        pulses = 0;
        computer.reset();
        refuelDetector.rearm();
        resets++;
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
