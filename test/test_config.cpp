#include "test_framework.h"
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

// Existing units were calibrated against the timer-polled counter, so an
// upgrade must not silently move them onto the interrupt counter.
TEST(Config, capture_mode_defaults_to_the_legacy_counter) {
  FakeEeprom e;
  Settings s(e);
  CHECK_EQ(s.captureMode(), CAPTURE_TIMER_POLL);
  CHECK(isValidCaptureMode(CAPTURE_TIMER_POLL));
  CHECK(isValidCaptureMode(CAPTURE_PIN_INTERRUPT));
  CHECK(!isValidCaptureMode(2));
  CHECK(!isValidCaptureMode(255));
  CHECK_EQ(validateCaptureMode(255), DEFAULT_CAPTURE_MODE);

  s.setCaptureMode(CAPTURE_PIN_INTERRUPT);
  CHECK_EQ(s.captureMode(), CAPTURE_PIN_INTERRUPT);
  s.setCaptureMode(99);
  CHECK_EQ(s.captureMode(), DEFAULT_CAPTURE_MODE);
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
  s.setCaptureMode(CAPTURE_PIN_INTERRUPT);

  CHECK_NEAR(s.fuelUsedGallons(), 60.0f, 1e-3);
  CHECK_EQ(s.canSpeed(), (uint8_t)18);
  CHECK_EQ(s.flowPin(), (uint8_t)30);
  CHECK(s.metric());
  CHECK_EQ(s.k(), MAX_K);
  CHECK_EQ(s.captureMode(), CAPTURE_PIN_INTERRUPT);
  CHECK(ADDR_K + 4 <= ADDR_CAPTURE_MODE);
  CHECK(ADDR_CAPTURE_MODE + 1 <= ADDR_END);
}
