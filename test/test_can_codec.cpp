#include "test_framework.h"
#include "../src/CanCodec.h"

using namespace fuel;

namespace {

// Fill a frame with a recognizable non-zero pattern, standing in for the stale
// stack contents the old code read past the reported length.
void poison(uint8_t* buf, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) buf[i] = 0xFF;
}

}  // namespace

// ---------------------------------------------------------------------------
// Receive path
// ---------------------------------------------------------------------------

// The other cause of unexplained resets. readMsgBufID only fills `len` bytes,
// but the old code read buf[5] unconditionally from an uninitialized local
// array, so leftover stack bytes were read as a reset command.
TEST(CanCodec, short_state_frame_cannot_request_a_reset) {
  Parameters p;
  uint8_t buf[8];
  poison(buf, 8);
  // A sender that only reports speed: 2 valid bytes, the rest stale.
  buf[0] = 0;
  buf[1] = 0;
  DecodeResult r = decodeStateFrame(buf, 2, false, p);
  CHECK(r.accepted);
  CHECK(!r.resetRequested);
}

TEST(CanCodec, reset_is_only_honored_when_its_byte_was_received) {
  Parameters p;
  uint8_t buf[8] = { 0, 0, 0, 0, 0, 1, 0, 0 };
  for (uint8_t len = 0; len < 6; len++) {
    DecodeResult r = decodeStateFrame(buf, len, false, p);
    CHECK(!r.resetRequested);
  }
  DecodeResult r = decodeStateFrame(buf, 6, false, p);
  CHECK(r.accepted);
  CHECK(r.resetRequested);
}

TEST(CanCodec, short_frame_leaves_later_fields_untouched) {
  Parameters p;
  p.fuelFull = false;
  p.fuelLevelGals = 7.5f;

  uint8_t buf[8];
  poison(buf, 8);
  buf[0] = 0;
  buf[1] = 0;
  decodeStateFrame(buf, 2, false, p);
  // fuelFull lives at byte 4 and was never received, so it must not flip to
  // true just because the buffer happened to hold 0xFF there.
  CHECK(!p.fuelFull);
  CHECK_NEAR(p.fuelLevelGals, 7.5f, 1e-6);
}

TEST(CanCodec, empty_or_null_frames_are_rejected) {
  Parameters p;
  uint8_t buf[8] = { 0 };
  CHECK(!decodeStateFrame(buf, 0, false, p).accepted);
  CHECK(!decodeStateFrame(buf, 1, false, p).accepted);
  CHECK(!decodeStateFrame(0, 8, false, p).accepted);
  CHECK(!decodeCapacityFrame(0, 8, false, p).accepted);
}

TEST(CanCodec, decodes_capacity_frame) {
  Parameters p;
  // 18.50 gal, 92.500 s lap, auto-reset on
  uint8_t buf[8] = { 0x07, 0x3A, 0x00, 0x01, 0x69, 0x64, 0x01, 0x00 };
  DecodeResult r = decodeCapacityFrame(buf, 8, false, p);
  CHECK(r.accepted);
  CHECK_NEAR(p.capacityGals, 18.50f, 1e-4);
  CHECK_EQ(p.lastLapMs, 92516u);
  CHECK(p.autoReset);
}

TEST(CanCodec, decodes_state_frame) {
  Parameters p;
  // 88.5 mph, 12.25 gal in the cell, full flag set, no reset
  uint8_t buf[8] = { 0x03, 0x75, 0x04, 0xC5, 0x01, 0x00, 0x00, 0x00 };
  DecodeResult r = decodeStateFrame(buf, 8, false, p);
  CHECK(r.accepted);
  CHECK_NEAR(p.speedMph, 88.5f, 1e-3);
  CHECK_NEAR(p.fuelLevelGals, 12.21f, 1e-3);
  CHECK(p.fuelFull);
  CHECK(!r.resetRequested);
}

TEST(CanCodec, boolean_fields_accept_any_non_zero) {
  Parameters p;
  uint8_t buf[8] = { 0, 0, 0, 0, 0x7F, 0x2A, 0, 0 };
  DecodeResult r = decodeStateFrame(buf, 8, false, p);
  CHECK(p.fuelFull);
  CHECK(r.resetRequested);
}

// A stuck-high bus or a mis-scaled sender used to push a 655 gallon capacity
// straight into the endurance maths.
TEST(CanCodec, implausible_capacity_is_clamped) {
  Parameters p;
  // 0xFFFF decodes to 655.35 gal, which would flow straight into the
  // endurance and laps-remaining maths.
  uint8_t buf[8] = { 0xFF, 0xFF, 0, 0, 0, 0, 0, 0 };
  decodeCapacityFrame(buf, 8, false, p);
  CHECK_NEAR(p.capacityGals, MAX_PLAUSIBLE_CAPACITY_GALS, 1e-4);
}

TEST(CanCodec, implausible_speed_is_clamped) {
  Parameters p;
  // 0xFFFF decodes to 6553.5 mph. Clamping high is the safe direction: the
  // refuel gate needs speed below 3, so a garbled frame cannot enable it.
  uint8_t buf[8] = { 0xFF, 0xFF, 0, 0, 0, 0, 0, 0 };
  decodeStateFrame(buf, 8, false, p);
  CHECK_NEAR(p.speedMph, MAX_PLAUSIBLE_SPEED_MPH, 1e-3);
}

TEST(CanCodec, metric_receive_converts_to_internal_units) {
  Parameters metricParams;
  Parameters imperialParams;
  // 100.00 on the wire, and 100.0 kph
  uint8_t cap[8] = { 0x27, 0x10, 0, 0, 0, 0, 0, 0 };
  decodeCapacityFrame(cap, 8, true, metricParams);
  decodeCapacityFrame(cap, 8, false, imperialParams);
  // 100 liters is about 26.42 gallons.
  CHECK_NEAR(metricParams.capacityGals, 26.4172f, 1e-3);
  CHECK_NEAR(imperialParams.capacityGals, 100.0f, 1e-3);

  uint8_t st[8] = { 0x03, 0xE8, 0, 0, 0, 0, 0, 0 };  // 100.0
  decodeStateFrame(st, 8, true, metricParams);
  CHECK_NEAR(metricParams.speedMph, 62.137f, 1e-2);
}

TEST(CanCodec, unknown_identifiers_are_ignored) {
  Parameters p;
  uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  DecodeResult r = decodeFrame(0x100B0099UL, buf, 8, false, p);
  CHECK(!r.accepted);
  CHECK(!r.resetRequested);
  CHECK_NEAR(p.capacityGals, 0.0f, 1e-6);
}

TEST(CanCodec, dispatch_routes_to_the_right_decoder) {
  Parameters p;
  uint8_t cap[8] = { 0x07, 0x3A, 0, 0, 0, 0, 1, 0 };
  CHECK(decodeFrame(CAN_ID_RX_CAPACITY, cap, 8, false, p).accepted);
  CHECK_NEAR(p.capacityGals, 18.50f, 1e-4);

  uint8_t st[8] = { 0x00, 0x00, 0, 0, 1, 1, 0, 0 };
  DecodeResult r = decodeFrame(CAN_ID_RX_STATE, st, 8, false, p);
  CHECK(r.accepted);
  CHECK(r.resetRequested);
  CHECK(p.fuelFull);
}

// ---------------------------------------------------------------------------
// Transmit path
// ---------------------------------------------------------------------------

TEST(CanCodec, encodes_usage_frame) {
  FuelStatus s;
  s.fuelPulses = 0x12345678UL;
  s.fuelUsedGals = 5.55f;
  s.fuelConsumptionGalMin = 0.1553f;

  TxFrames f;
  encodeStatus(s, false, f);
  CHECK_EQ(readU32(&f.usage[0]), 0x12345678UL);
  CHECK_EQ(readU16(&f.usage[4]), (uint16_t)555);   // gal x100
  CHECK_EQ(readU16(&f.usage[6]), (uint16_t)1553);  // gal/min x10000
}

TEST(CanCodec, encodes_remaining_frame) {
  FuelStatus s;
  s.fuelRemainingGals = 8.21f;
  s.fuelRemainingMins = 33.0f;
  s.lapsRemaining = 21;
  s.fuelConsumptionGalLap = 0.41f;

  TxFrames f;
  encodeStatus(s, false, f);
  CHECK_EQ(readU16(&f.remaining[0]), (uint16_t)821);
  CHECK_EQ(readU16(&f.remaining[2]), (uint16_t)1980);  // 33 min in seconds
  CHECK_EQ(readU16(&f.remaining[4]), (uint16_t)21);
  CHECK_EQ(readU16(&f.remaining[6]), (uint16_t)4100);  // gal/lap x10000
}

TEST(CanCodec, encodes_liters_per_lap_frame) {
  FuelStatus s;
  s.fuelConsumptionGalLap = 0.41f;
  TxFrames f;
  encodeStatus(s, false, f);
  // 0.41 gal is 1.5520 liters.
  CHECK_EQ(readU16(&f.metricLap[0]), (uint16_t)15520);
}

TEST(CanCodec, metric_transmit_uses_liters_and_cc_per_second) {
  FuelStatus s;
  s.fuelUsedGals = 1.0f;
  s.fuelRemainingGals = 2.0f;
  s.fuelConsumptionGalMin = 1.0f;

  TxFrames f;
  encodeStatus(s, true, f);
  CHECK_EQ(readU16(&f.usage[4]), (uint16_t)378);   // 3.7854 L x100
  CHECK_EQ(readU16(&f.usage[6]), (uint16_t)63);    // 63.09 cc/sec
  CHECK_EQ(readU16(&f.remaining[0]), (uint16_t)757);  // 7.5708 L x100
}

// Casting an out-of-range float straight to uint16 is undefined behavior and
// wrapped to a small number on AVR - a nearly empty tank could read full.
TEST(CanCodec, oversized_values_saturate_instead_of_wrapping) {
  FuelStatus s;
  s.fuelUsedGals = 5000.0f;             // x100 is far past 65535
  s.fuelRemainingGals = 5000.0f;
  s.fuelConsumptionGalMin = 900.0f;     // x10000 likewise
  s.fuelRemainingMins = 100000.0f;
  s.fuelConsumptionGalLap = 50.0f;
  s.lapsRemaining = 30000;

  TxFrames f;
  encodeStatus(s, false, f);
  CHECK_EQ(readU16(&f.usage[4]), (uint16_t)65535);
  CHECK_EQ(readU16(&f.usage[6]), (uint16_t)65535);
  CHECK_EQ(readU16(&f.remaining[0]), (uint16_t)65535);
  CHECK_EQ(readU16(&f.remaining[2]), (uint16_t)65535);
  CHECK_EQ(readU16(&f.remaining[6]), (uint16_t)65535);
  CHECK_EQ(readU16(&f.metricLap[0]), (uint16_t)65535);
}

TEST(CanCodec, negative_and_nan_values_encode_as_zero) {
  FuelStatus s;
  const float nan = 0.0f / 0.0f;
  s.fuelUsedGals = -5.0f;
  s.fuelRemainingGals = nan;
  s.fuelConsumptionGalMin = -1.0f;
  s.fuelRemainingMins = nan;
  s.fuelConsumptionGalLap = -2.0f;
  s.lapsRemaining = -7;

  TxFrames f;
  encodeStatus(s, false, f);
  CHECK_EQ(readU16(&f.usage[4]), (uint16_t)0);
  CHECK_EQ(readU16(&f.usage[6]), (uint16_t)0);
  CHECK_EQ(readU16(&f.remaining[0]), (uint16_t)0);
  CHECK_EQ(readU16(&f.remaining[2]), (uint16_t)0);
  CHECK_EQ(readU16(&f.remaining[4]), (uint16_t)0);
  CHECK_EQ(readU16(&f.remaining[6]), (uint16_t)0);
  CHECK_EQ(readU16(&f.metricLap[0]), (uint16_t)0);
}

TEST(CanCodec, unused_frame_bytes_are_zeroed) {
  FuelStatus s;  // all zero
  TxFrames f;
  poison(f.usage, 8);
  poison(f.remaining, 8);
  poison(f.metricLap, 8);
  encodeStatus(s, false, f);
  // Nothing from a previous frame may survive anywhere in any of the three.
  for (uint8_t i = 0; i < 8; i++) {
    CHECK_EQ(f.usage[i], (uint8_t)0);
    CHECK_EQ(f.remaining[i], (uint8_t)0);
    CHECK_EQ(f.metricLap[i], (uint8_t)0);
  }
}

// readMsgBufID reports the raw 4-bit DLC, which can be up to 15 even though
// the payload buffer only holds 8. The decoders must not read past the buffer.
TEST(CanCodec, over_long_reported_length_is_handled) {
  Parameters p;
  uint8_t buf[8] = { 0x07, 0x3A, 0, 0, 1, 1, 1, 0 };
  DecodeResult r = decodeStateFrame(buf, 15, false, p);
  CHECK(r.accepted);
  CHECK(r.resetRequested);
  CHECK(decodeCapacityFrame(buf, 15, false, p).accepted);
}

TEST(CanCodec, byte_order_helpers_are_big_endian) {
  uint8_t buf[4] = { 0, 0, 0, 0 };
  writeU16(buf, 0xABCD);
  CHECK_EQ(buf[0], (uint8_t)0xAB);
  CHECK_EQ(buf[1], (uint8_t)0xCD);
  CHECK_EQ(readU16(buf), (uint16_t)0xABCD);

  writeU32(buf, 0x01020304UL);
  CHECK_EQ(buf[0], (uint8_t)0x01);
  CHECK_EQ(buf[3], (uint8_t)0x04);
  CHECK_EQ(readU32(buf), 0x01020304UL);
}

// ---------------------------------------------------------------------------
// Diagnostics, carried in the unused bytes of 0x100B0005
// ---------------------------------------------------------------------------

// Those bytes were always zero before, so anything already decoding the frame
// must be unaffected when there is nothing to report.
TEST(CanCodec, diagnostics_are_absent_when_nothing_is_set) {
  FuelStatus s;
  s.fuelConsumptionGalLap = 0.41f;
  TxFrames f;
  encodeStatus(s, false, f);
  CHECK_EQ(readU16(&f.metricLap[0]), (uint16_t)15520);
  for (uint8_t i = 2; i < 8; i++) CHECK_EQ(f.metricLap[i], (uint8_t)0);
}

TEST(CanCodec, diagnostics_do_not_disturb_the_liters_per_lap_value) {
  FuelStatus s;
  s.fuelConsumptionGalLap = 0.41f;
  Diagnostics d;
  d.refuelArmed = true;
  d.resetCount = 7;
  d.uptimeMs = 123456;
  TxFrames f;
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[0]), (uint16_t)15520);
}

TEST(CanCodec, encodes_each_diagnostic_flag_independently) {
  Diagnostics d;
  CHECK_EQ(encodeDiagFlags(d), (uint8_t)0);

  d = Diagnostics();
  d.refuelArmed = true;
  CHECK_EQ(encodeDiagFlags(d), DIAG_REFUEL_ARMED);

  d = Diagnostics();
  d.refuelDwelling = true;
  CHECK_EQ(encodeDiagFlags(d), DIAG_REFUEL_DWELLING);

  d = Diagnostics();
  d.interruptCapture = true;
  CHECK_EQ(encodeDiagFlags(d), DIAG_INTERRUPT_CAPTURE);

  d = Diagnostics();
  d.stateFresh = true;
  CHECK_EQ(encodeDiagFlags(d), DIAG_STATE_FRESH);

  d = Diagnostics();
  d.canError = true;
  CHECK_EQ(encodeDiagFlags(d), DIAG_CAN_ERROR);

  // All at once, so no two share a bit.
  d = Diagnostics();
  d.refuelArmed = d.refuelDwelling = d.interruptCapture = true;
  d.stateFresh = d.canError = true;
  CHECK_EQ(encodeDiagFlags(d), (uint8_t)0x1F);
}

TEST(CanCodec, encodes_reset_count_and_reason) {
  FuelStatus s;
  Diagnostics d;
  d.resetCount = 3;
  d.lastResetReason = RESET_REFUEL;
  TxFrames f;
  encodeStatus(s, false, d, f);
  CHECK_EQ(f.metricLap[3], (uint8_t)3);
  CHECK_EQ(f.metricLap[4], RESET_REFUEL);
}

// The reason a counter dropped cannot be told from a reboot without this.
TEST(CanCodec, encodes_uptime_in_seconds) {
  FuelStatus s;
  Diagnostics d;
  TxFrames f;

  d.uptimeMs = 0;
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[5]), (uint16_t)0);

  d.uptimeMs = 45678;                    // 45.678 s
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[5]), (uint16_t)45);

  d.uptimeMs = 3600000UL;                // one hour
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[5]), (uint16_t)3600);
}

TEST(CanCodec, uptime_saturates_rather_than_wrapping) {
  FuelStatus s;
  Diagnostics d;
  TxFrames f;
  d.uptimeMs = 65535000UL;               // exactly the ceiling
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[5]), (uint16_t)65535);
  d.uptimeMs = 4294000000UL;             // near the millis() rollover
  encodeStatus(s, false, d, f);
  CHECK_EQ(readU16(&f.metricLap[5]), (uint16_t)65535);
}

// Exact while small, because the question is usually "any noise at all?".
TEST(CanCodec, rejected_edges_are_exact_then_pinned) {
  FuelStatus s;
  Diagnostics d;
  TxFrames f;

  d.rejectedEdges = 0;
  encodeStatus(s, false, d, f);
  CHECK_EQ(f.metricLap[7], (uint8_t)0);

  d.rejectedEdges = 17;
  encodeStatus(s, false, d, f);
  CHECK_EQ(f.metricLap[7], (uint8_t)17);

  d.rejectedEdges = 255;
  encodeStatus(s, false, d, f);
  CHECK_EQ(f.metricLap[7], (uint8_t)255);

  d.rejectedEdges = 1000000UL;
  encodeStatus(s, false, d, f);
  CHECK_EQ(f.metricLap[7], (uint8_t)255);
}

// A reboot and a reset must look different in a log.
TEST(CanCodec, a_reboot_is_distinguishable_from_a_reset) {
  FuelStatus s;
  TxFrames before, after;

  Diagnostics running;
  running.uptimeMs = 1800000UL;          // half an hour in
  running.resetCount = 0;
  encodeStatus(s, false, running, before);

  Diagnostics rebooted;                  // uptime back to nearly zero
  rebooted.uptimeMs = 800;
  rebooted.resetCount = 0;
  encodeStatus(s, false, rebooted, after);

  CHECK(readU16(&before.metricLap[5]) > readU16(&after.metricLap[5]));
  // A reset instead leaves uptime climbing and bumps the count.
  Diagnostics reset;
  reset.uptimeMs = 1800500UL;
  reset.resetCount = 1;
  reset.lastResetReason = RESET_REFUEL;
  TxFrames afterReset;
  encodeStatus(s, false, reset, afterReset);
  CHECK(readU16(&afterReset.metricLap[5]) >= readU16(&before.metricLap[5]));
  CHECK(afterReset.metricLap[3] > before.metricLap[3]);
}

// A reset command that was received and deliberately ignored has to be
// visible in a log, so a counter that did NOT reset can be explained too.
TEST(CanCodec, encodes_the_ignored_can_reset_flag) {
  Diagnostics d;
  CHECK_EQ(encodeDiagFlags(d) & DIAG_CAN_RESET_IGNORED, (uint8_t)0);
  d.canResetIgnored = true;
  CHECK_EQ(encodeDiagFlags(d) & DIAG_CAN_RESET_IGNORED, DIAG_CAN_RESET_IGNORED);
  // ...and it does not collide with any other flag.
  Diagnostics all;
  all.refuelArmed = all.refuelDwelling = all.interruptCapture = true;
  all.stateFresh = all.canError = all.canResetIgnored = true;
  CHECK_EQ(encodeDiagFlags(all), (uint8_t)0x3F);
}
