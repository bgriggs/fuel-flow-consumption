#include "test_framework.h"
#include "../src/CanCodec.h"
#include "../src/Config.h"

using namespace fuel;

namespace {

// Stands in for the AVR EEPROM. Starts erased, which is the state the bug
// reports were most sensitive to.
class FakeEeprom : public EepromStore {
public:
  FakeEeprom() { erase(); }

  void erase() {
    for (int i = 0; i < 64; i++) cells_[i] = 0xFF;
    writeCount_ = 0;
  }
  uint8_t read(uint16_t addr) { return cells_[addr]; }
  void update(uint16_t addr, uint8_t value) {
    // Mirrors EEPROM.update(): only a changed byte costs a write cycle.
    if (cells_[addr] != value) {
      cells_[addr] = value;
      writeCount_++;
    }
  }
  void poke(uint16_t addr, uint8_t value) { cells_[addr] = value; }
  int writeCount() const { return writeCount_; }

private:
  uint8_t cells_[64];
  int writeCount_;
};

}  // namespace

// ---------------------------------------------------------------------------
// Erased EEPROM
// ---------------------------------------------------------------------------

// Every one of these read back as 0xFF on a fresh board. Used raw that meant
// pinMode(255) (out of bounds for the pin tables), a K-factor of 4294967295,
// a CAN speed no driver accepts, and metric mode switched on.
TEST(Config, erased_eeprom_yields_safe_defaults) {
  FakeEeprom e;
  Settings s(e);
  CHECK_EQ(s.canSpeed(), DEFAULT_CAN_SPEED);
  CHECK_EQ(s.flowPin(), DEFAULT_FLOW_PIN);
  CHECK_EQ(s.k(), DEFAULT_K);
  CHECK(!s.metric());
  CHECK_EQ(s.captureMode(), DEFAULT_CAPTURE_MODE);
  CHECK_NEAR(s.fuelUsedGallons(), 0.0f, 1e-6);
}

// Polling tops out at 25.9 GPH against a transducer rated to 70, so hardware
// interrupt counting is the default and polling is only a fallback.
TEST(Config, capture_mode_defaults_to_the_interrupt_counter) {
  FakeEeprom e;
  Settings s(e);
  CHECK_EQ(s.captureMode(), CAPTURE_PIN_INTERRUPT);
  CHECK(isValidCaptureMode(CAPTURE_TIMER_POLL));
  CHECK(isValidCaptureMode(CAPTURE_PIN_INTERRUPT));
  CHECK(!isValidCaptureMode(2));
  CHECK(!isValidCaptureMode(255));
  CHECK_EQ(validateCaptureMode(255), DEFAULT_CAPTURE_MODE);

  s.setCaptureMode(CAPTURE_TIMER_POLL);
  CHECK_EQ(s.captureMode(), CAPTURE_TIMER_POLL);
  s.setCaptureMode(99);
  CHECK_EQ(s.captureMode(), DEFAULT_CAPTURE_MODE);
}

// A K dialed down to hide a counter that was losing pulses will over-read
// once every pulse is counted, so the firmware says so at boot.
TEST(Config, recognizes_a_k_far_from_the_transducer_nominal) {
  CHECK(isNearNominalK(DEFAULT_K));
  CHECK(isNearNominalK(63000));   // within 10%, plausible installation effect
  CHECK(!isNearNominalK(45000));  // compensating for ~34% lost pulses
  CHECK(!isNearNominalK(0));
  CHECK(!isNearNominalK(MIN_K));
  CHECK(!isNearNominalK(MAX_K));
}

// Pin the tolerance at its exact edges. Without these the band could be
// widened or narrowed without any test noticing.
TEST(Config, k_tolerance_band_is_exact) {
  CHECK_EQ(K_TOLERANCE_PERCENT, 10u);
  CHECK(isNearNominalK(61200));   // DEFAULT_K - 10%
  CHECK(!isNearNominalK(61199));
  CHECK(isNearNominalK(74800));   // DEFAULT_K + 10%
  CHECK(!isNearNominalK(74801));
}

// The holdoff is the only noise control in interrupt capture, so its bounds
// and its effect on the countable flow rate need pinning.
TEST(Config, pulse_guard_is_range_checked) {
  CHECK(isValidPulseGuard(0));                        // holdoff disabled
  CHECK(isValidPulseGuard(DEFAULT_PULSE_GUARD_UNITS));
  CHECK(isValidPulseGuard(MAX_PULSE_GUARD_UNITS));
  CHECK(!isValidPulseGuard(MAX_PULSE_GUARD_UNITS + 1));
  CHECK(!isValidPulseGuard(255));                     // erased EEPROM
  CHECK_EQ(validatePulseGuard(255), DEFAULT_PULSE_GUARD_UNITS);
  CHECK_EQ(validatePulseGuard(0), (uint8_t)0);
}

TEST(Config, erased_eeprom_gives_the_default_pulse_guard) {
  FakeEeprom e;
  Settings s(e);
  CHECK_EQ(s.pulseGuardUnits(), DEFAULT_PULSE_GUARD_UNITS);
  CHECK_EQ(s.pulseGuardUs(), (uint16_t)400);
  s.setPulseGuardUnits(80);
  CHECK_EQ(s.pulseGuardUs(), (uint16_t)800);
}

// Written straight into the cell, bypassing the setter, so this pins the
// coercion done on read rather than the coercion done on write.
TEST(Config, an_out_of_range_stored_guard_is_coerced_on_read) {
  FakeEeprom e;
  Settings s(e);
  e.poke(ADDR_PULSE_GUARD, 255);
  CHECK_EQ(s.pulseGuardUnits(), DEFAULT_PULSE_GUARD_UNITS);
  e.poke(ADDR_PULSE_GUARD, (uint8_t)(MAX_PULSE_GUARD_UNITS + 1));
  CHECK_EQ(s.pulseGuardUnits(), DEFAULT_PULSE_GUARD_UNITS);
  e.poke(ADDR_PULSE_GUARD, 0);
  CHECK_EQ(s.pulseGuardUnits(), (uint8_t)0);  // 0 is a legitimate setting
}

// The default has to pass an FT60 at its rated 70 GPH (1322 Hz) with margin,
// while still rejecting everything meaningfully faster.
TEST(Config, default_pulse_guard_passes_the_transducer_at_full_rate) {
  uint16_t ceiling = countableRateHz(DEFAULT_PULSE_GUARD_UNITS *
                                     PULSE_GUARD_UNIT_US);
  CHECK_EQ(ceiling, (uint16_t)2500);
  // Derive the FT60 rate rather than hard-coding it, so the margin is still
  // checked if the nominal K ever changes.
  const uint32_t ft60MaxHz = DEFAULT_K * 70UL / 3600UL;  // 70 GPH
  CHECK_EQ(ft60MaxHz, 1322UL);
  // The margin has to be real, not marginal.
  CHECK((uint32_t)ceiling > ft60MaxHz * 3UL / 2UL);
}

TEST(Config, countable_rate_reports_no_ceiling_when_the_guard_is_off) {
  CHECK_EQ(countableRateHz(0), (uint16_t)0);
  CHECK_EQ(countableRateHz(1000), (uint16_t)1000);
  CHECK_EQ(countableRateHz(2000), (uint16_t)500);
}

// The smallest settable guard implies 100000 Hz, which does not fit a uint16
// and wrapped to 34464 before it was clamped, so status reported a countable
// ceiling far below the truth.
TEST(Config, countable_rate_saturates_instead_of_wrapping) {
  CHECK_EQ(countableRateHz(10), (uint16_t)65535);
  CHECK_EQ(countableRateHz(16), (uint16_t)62500);
  CHECK_EQ(countableRateHz(20), (uint16_t)50000);
  CHECK(countableRateHz(10) >= countableRateHz(20));
}

// The recalibration advice has to stop once a K has been chosen deliberately,
// or it nags forever, including at someone who has just recalibrated
// correctly, since a right answer often lands outside the FT60 band.
TEST(Config, k_acknowledgement_is_sticky_and_starts_clear) {
  FakeEeprom e;
  Settings s(e);
  CHECK(!s.kAcknowledged());  // erased EEPROM
  s.setKAcknowledged();
  CHECK(s.kAcknowledged());
  s.setKAcknowledged();
  CHECK(s.kAcknowledged());
}

// The worked example in the README lands outside the band, so it must be
// possible to stop the advice after following it.
TEST(Config, a_correctly_recalibrated_k_can_be_acknowledged) {
  FakeEeprom e;
  Settings s(e);
  const uint32_t recalibrated = 58286;  // 68000 x 18/21, from the README
  CHECK(!isNearNominalK(recalibrated));
  CHECK(!s.kAcknowledged());
  s.setK(recalibrated);
  s.setKAcknowledged();
  CHECK(s.kAcknowledged());
  CHECK_EQ(s.k(), recalibrated);
}

// Distinguishing an upgrade from a deliberate choice is what stops the
// recalibration warning nagging someone who is not on an FT60.
TEST(Config, capture_mode_defaulted_flag_tracks_whether_it_was_written) {
  FakeEeprom e;
  Settings s(e);
  CHECK(s.captureModeWasDefaulted());
  s.setCaptureMode(CAPTURE_PIN_INTERRUPT);
  CHECK(!s.captureModeWasDefaulted());
  s.setCaptureMode(CAPTURE_TIMER_POLL);
  CHECK(!s.captureModeWasDefaulted());
}

TEST(Config, erased_eeprom_is_reported_as_defaulted) {
  FakeEeprom e;
  Settings s(e);
  CHECK(s.canSpeedWasDefaulted());
  CHECK(s.flowPinWasDefaulted());
  CHECK(s.kWasDefaulted());
}

TEST(Config, configured_values_are_not_reported_as_defaulted) {
  FakeEeprom e;
  Settings s(e);
  s.setCanSpeed(15);
  s.setFlowPin(2);
  s.setK(68000);
  CHECK(!s.canSpeedWasDefaulted());
  CHECK(!s.flowPinWasDefaulted());
  CHECK(!s.kWasDefaulted());
}

// ---------------------------------------------------------------------------
// Existing devices
// ---------------------------------------------------------------------------

// The byte layout is unchanged, so a unit already in service keeps its
// settings when this firmware is flashed over the old one.
TEST(Config, reads_the_legacy_byte_layout) {
  FakeEeprom e;
  e.poke(ADDR_CAN_SPEED, 16);
  e.poke(ADDR_FLOW_PIN, 0);
  e.poke(ADDR_METRIC, 1);
  e.poke(ADDR_K, 0x00);  // 68000 == 0x000109A0
  e.poke(ADDR_K + 1, 0x01);
  e.poke(ADDR_K + 2, 0x09);
  e.poke(ADDR_K + 3, 0xA0);
  e.poke(ADDR_FUEL_USED, 0x1B);  // 7000 == 7.000 gal
  e.poke(ADDR_FUEL_USED + 1, 0x58);

  Settings s(e);
  CHECK_EQ(s.canSpeed(), (uint8_t)16);
  CHECK_EQ(s.flowPin(), (uint8_t)0);
  CHECK(s.metric());
  CHECK_EQ(s.k(), 68000UL);
  CHECK_NEAR(s.fuelUsedGallons(), 7.0f, 1e-4);
}

// ---------------------------------------------------------------------------
// Validation of individual fields
// ---------------------------------------------------------------------------

TEST(Config, can_speed_is_range_checked) {
  CHECK(!isValidCanSpeed(0));
  CHECK(isValidCanSpeed(1));
  CHECK(isValidCanSpeed(18));
  CHECK(!isValidCanSpeed(19));
  CHECK(!isValidCanSpeed(255));
  CHECK_EQ(validateCanSpeed(0), DEFAULT_CAN_SPEED);
  CHECK_EQ(validateCanSpeed(255), DEFAULT_CAN_SPEED);
  CHECK_EQ(validateCanSpeed(16), (uint8_t)16);
}

TEST(Config, flow_pin_is_range_checked) {
  CHECK(isValidFlowPin(0));
  CHECK(isValidFlowPin(MAX_FLOW_PIN));
  CHECK(!isValidFlowPin(MAX_FLOW_PIN + 1));
  CHECK(!isValidFlowPin(255));
  CHECK_EQ(validateFlowPin(255), DEFAULT_FLOW_PIN);
  CHECK_EQ(validateFlowPin(7), (uint8_t)7);
}

// bool metricUnits = EEPROM.read(4) made an erased 0xFF true, so a fresh board
// came up in metric mode. Other non-zero values stay metric, because the old
// setmetric accepted any number and a unit configured that way must not flip
// units on upgrade.
TEST(Config, metric_flag_rejects_the_erased_pattern_but_keeps_legacy_values) {
  CHECK(!validateMetric(0));
  CHECK(validateMetric(1));
  CHECK(validateMetric(2));
  CHECK(!validateMetric(255));
}

TEST(Config, k_is_range_checked) {
  CHECK(!isValidK(0));
  CHECK(!isValidK(MIN_K - 1));
  CHECK(isValidK(MIN_K));
  CHECK(isValidK(68000));
  CHECK(isValidK(MAX_K));
  CHECK(!isValidK(MAX_K + 1));
  CHECK(!isValidK(0xFFFFFFFFUL));
  CHECK_EQ(validateK(0), DEFAULT_K);
  CHECK_EQ(validateK(0xFFFFFFFFUL), DEFAULT_K);
  CHECK_EQ(validateK(68000), 68000UL);
}

TEST(Config, settings_reject_out_of_range_writes) {
  FakeEeprom e;
  Settings s(e);
  s.setK(0);
  CHECK_EQ(s.k(), DEFAULT_K);
  s.setCanSpeed(99);
  CHECK_EQ(s.canSpeed(), DEFAULT_CAN_SPEED);
  s.setFlowPin(200);
  CHECK_EQ(s.flowPin(), DEFAULT_FLOW_PIN);
}

// ---------------------------------------------------------------------------
// Fuel-used persistence
// ---------------------------------------------------------------------------

// 0xFFFF literally means 65.535 gallons, which the old code restored on a
// fresh board as if that much fuel had already been burned.
TEST(Config, unprogrammed_fuel_cell_decodes_to_zero) {
  CHECK_NEAR(decodeStoredGallons(STORED_GALLONS_UNPROGRAMMED), 0.0f, 1e-6);
  CHECK_NEAR(decodeStoredGallons(0), 0.0f, 1e-6);
  CHECK_NEAR(decodeStoredGallons(1234), 1.234f, 1e-6);
}

// Saving is capped just below the all-ones pattern so a genuine reading can
// never be mistaken for an erased cell, and so gallons x1000 cannot wrap.
TEST(Config, stored_gallons_saturate_below_the_erased_pattern) {
  // Without the clamp, 1000 gal x1000 wraps the uint16 to a small number.
  CHECK_EQ(encodeStoredGallons(1000.0f), (uint16_t)65534);
  CHECK_EQ(encodeStoredGallons(65.535f), (uint16_t)65534);
  CHECK_EQ(encodeStoredGallons(-1.0f), (uint16_t)0);
  CHECK_EQ(encodeStoredGallons(0.0f / 0.0f), (uint16_t)0);
}

TEST(Config, fuel_used_round_trips) {
  FakeEeprom e;
  Settings s(e);
  s.setFuelUsedGallons(12.345f);
  CHECK_NEAR(s.fuelUsedGallons(), 12.345f, 1e-3);
  s.setFuelUsedGallons(0.0f);
  CHECK_NEAR(s.fuelUsedGallons(), 0.0f, 1e-6);
}

// The cell is rewritten every 30 s for the life of the car, against a 100k
// cycle rating, so unchanged values must not burn a write.
TEST(Config, saving_an_unchanged_value_costs_no_write_cycle) {
  FakeEeprom e;
  Settings s(e);
  s.setFuelUsedGallons(5.0f);
  int after = e.writeCount();
  s.setFuelUsedGallons(5.0f);
  s.setFuelUsedGallons(5.0f);
  CHECK_EQ(e.writeCount(), after);
}

TEST(Config, all_settings_round_trip_together) {
  FakeEeprom e;
  Settings s(e);
  s.setCanSpeed(13);
  s.setFlowPin(2);
  s.setMetric(true);
  s.setK(123456);
  s.setFuelUsedGallons(3.75f);

  CHECK_EQ(s.canSpeed(), (uint8_t)13);
  CHECK_EQ(s.flowPin(), (uint8_t)2);
  CHECK(s.metric());
  CHECK_EQ(s.k(), 123456UL);
  CHECK_NEAR(s.fuelUsedGallons(), 3.75f, 1e-3);

  s.setMetric(false);
  CHECK(!s.metric());
}

// Each setting must occupy its own bytes; an overlap would have one write
// corrupt another field.
TEST(Config, fields_do_not_overlap) {
  FakeEeprom e;
  Settings s(e);
  s.setFuelUsedGallons(60.0f);
  s.setCanSpeed(18);
  s.setFlowPin(30);
  s.setMetric(true);
  s.setK(MAX_K);
  s.setCaptureMode(CAPTURE_TIMER_POLL);
  s.setPulseGuardUnits(MAX_PULSE_GUARD_UNITS);
  s.setKAcknowledged();

  CHECK_NEAR(s.fuelUsedGallons(), 60.0f, 1e-3);
  CHECK_EQ(s.canSpeed(), (uint8_t)18);
  CHECK_EQ(s.flowPin(), (uint8_t)30);
  CHECK(s.metric());
  CHECK_EQ(s.k(), MAX_K);
  CHECK_EQ(s.captureMode(), CAPTURE_TIMER_POLL);
  CHECK(ADDR_K + 4 <= ADDR_CAPTURE_MODE);
  CHECK_EQ(s.pulseGuardUnits(), MAX_PULSE_GUARD_UNITS);
  CHECK(s.kAcknowledged());
  CHECK(ADDR_CAPTURE_MODE + 1 <= ADDR_PULSE_GUARD);
  CHECK(ADDR_PULSE_GUARD + 1 <= ADDR_K_ACKED);
  CHECK(ADDR_K_ACKED + 1 <= ADDR_END);
}

// ---------------------------------------------------------------------------
// Reset history, so a session can be explained after the fact
// ---------------------------------------------------------------------------

TEST(Config, erased_eeprom_reports_no_reset_history) {
  FakeEeprom e;
  Settings s(e);
  CHECK_EQ(s.lastResetReason(), RESET_NONE);
  CHECK_EQ(s.resetCount(), (uint8_t)0);  // 0xFF means never written
}

TEST(Config, records_the_reason_and_counts_resets) {
  FakeEeprom e;
  Settings s(e);
  s.recordReset(RESET_COMMANDED);
  CHECK_EQ(s.lastResetReason(), RESET_COMMANDED);
  CHECK_EQ(s.resetCount(), (uint8_t)1);

  s.recordReset(RESET_REFUEL);
  CHECK_EQ(s.lastResetReason(), RESET_REFUEL);
  CHECK_EQ(s.resetCount(), (uint8_t)2);
}

TEST(Config, an_unknown_stored_reason_reads_as_none) {
  FakeEeprom e;
  Settings s(e);
  e.poke(ADDR_LAST_RESET, 200);
  CHECK_EQ(s.lastResetReason(), RESET_NONE);
  e.poke(ADDR_LAST_RESET, 255);
  CHECK_EQ(s.lastResetReason(), RESET_NONE);
  // Checked against the raw cell: lastResetReason() sanitizes on read, so
  // asserting through it cannot tell whether recordReset validates at all.
  s.recordReset(200);
  CHECK_EQ(e.read(ADDR_LAST_RESET), (uint8_t)RESET_NONE);
  CHECK_EQ(s.lastResetReason(), RESET_NONE);
}

// It must never wrap round to look like a fresh device.
TEST(Config, reset_count_saturates_below_the_erased_pattern) {
  FakeEeprom e;
  Settings s(e);
  e.poke(ADDR_RESET_COUNT, (uint8_t)(MAX_RESET_COUNT - 1));
  s.recordReset(RESET_REFUEL);
  CHECK_EQ(s.resetCount(), MAX_RESET_COUNT);
  s.recordReset(RESET_REFUEL);
  CHECK_EQ(s.resetCount(), MAX_RESET_COUNT);
  CHECK(MAX_RESET_COUNT < 255);
}

TEST(Config, reset_history_does_not_overlap_other_settings) {
  FakeEeprom e;
  Settings s(e);
  s.setK(68000);
  s.setKAcknowledged();
  s.setPulseGuardUnits(40);
  s.recordReset(RESET_REFUEL);

  CHECK_EQ(s.k(), 68000UL);
  CHECK(s.kAcknowledged());
  CHECK_EQ(s.pulseGuardUnits(), (uint8_t)40);
  CHECK_EQ(s.lastResetReason(), RESET_REFUEL);
  CHECK_EQ(s.resetCount(), (uint8_t)1);
  CHECK(ADDR_K_ACKED + 1 <= ADDR_LAST_RESET);
  CHECK(ADDR_LAST_RESET + 1 <= ADDR_RESET_COUNT);
  CHECK(ADDR_RESET_COUNT + 1 <= ADDR_END);
}

// ---------------------------------------------------------------------------
// Honoring the bus reset command
// ---------------------------------------------------------------------------

// Enabled by default, because that is how the protocol has always worked; an
// upgrade must not silently stop obeying a dash that relies on it.
TEST(Config, can_reset_is_enabled_on_an_erased_eeprom) {
  FakeEeprom e;
  Settings s(e);
  CHECK(s.canResetEnabled());
}

TEST(Config, can_reset_can_be_turned_off_and_back_on) {
  FakeEeprom e;
  Settings s(e);
  s.setCanResetEnabled(false);
  CHECK(!s.canResetEnabled());
  s.setCanResetEnabled(true);
  CHECK(s.canResetEnabled());
}

// Only an explicit stored 0 disables it. A corrupt byte must not silently
// leave the device ignoring reset commands.
TEST(Config, only_an_explicit_zero_disables_the_can_reset) {
  CHECK(!validateCanResetEnabled(0));
  // Pinned independently of DEFAULT_CAN_RESET_ENABLED: if that default is ever
  // flipped, a stored 1 must still mean enabled.
  CHECK(validateCanResetEnabled(1));
  CHECK(validateCanResetEnabled(2));
  CHECK(validateCanResetEnabled(255));

  FakeEeprom e;
  Settings s(e);
  e.poke(ADDR_CAN_RESET, 200);
  CHECK(s.canResetEnabled());
  e.poke(ADDR_CAN_RESET, 0);
  CHECK(!s.canResetEnabled());
}

TEST(Config, can_reset_flag_has_its_own_byte) {
  FakeEeprom e;
  Settings s(e);
  s.setCanResetEnabled(false);
  s.recordReset(RESET_COMMANDED);
  s.setKAcknowledged();
  CHECK(!s.canResetEnabled());
  CHECK_EQ(s.lastResetReason(), RESET_COMMANDED);
  CHECK_EQ(s.resetCount(), (uint8_t)1);
  CHECK(ADDR_RESET_COUNT + 1 <= ADDR_CAN_RESET);
  CHECK(ADDR_CAN_RESET + 1 <= ADDR_END);
}
