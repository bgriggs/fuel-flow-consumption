#include "Config.h"
#include "CanCodec.h"
#include "FuelMath.h"

namespace fuel {

bool isValidCanSpeed(uint8_t raw) {
  return raw >= MIN_CAN_SPEED && raw <= MAX_CAN_SPEED;
}
bool isValidFlowPin(uint8_t raw) { return raw <= MAX_FLOW_PIN; }
bool isValidMetric(uint8_t raw) { return raw <= 1; }
bool isValidK(uint32_t raw) { return raw >= MIN_K && raw <= MAX_K; }
bool isValidCaptureMode(uint8_t raw) { return raw <= CAPTURE_PIN_INTERRUPT; }

uint8_t validateCanSpeed(uint8_t raw) {
  return isValidCanSpeed(raw) ? raw : DEFAULT_CAN_SPEED;
}
uint8_t validateFlowPin(uint8_t raw) {
  return isValidFlowPin(raw) ? raw : DEFAULT_FLOW_PIN;
}
// The old firmware assigned this byte straight to a bool, so any non-zero
// value meant metric, and its setmetric accepted any number. Preserve that for
// deliberately stored values, but exclude the erased pattern - reading 0xFF as
// "true" is what put a fresh board into metric mode by default.
bool validateMetric(uint8_t raw) {
  return raw != 0 && raw != 0xFF;
}
uint32_t validateK(uint32_t raw) { return isValidK(raw) ? raw : DEFAULT_K; }
bool isNearNominalK(uint32_t k) {
  uint32_t margin = (DEFAULT_K / 100UL) * K_TOLERANCE_PERCENT;
  return k >= DEFAULT_K - margin && k <= DEFAULT_K + margin;
}

bool isValidPulseGuard(uint8_t raw) { return raw <= MAX_PULSE_GUARD_UNITS; }
uint8_t validatePulseGuard(uint8_t raw) {
  return isValidPulseGuard(raw) ? raw : DEFAULT_PULSE_GUARD_UNITS;
}

uint16_t countableRateHz(uint16_t guardUs) {
  if (guardUs == 0) return 0;  // no holdoff, no ceiling
  uint32_t hz = 1000000UL / (uint32_t)guardUs;
  return hz > 65535UL ? (uint16_t)65535 : (uint16_t)hz;
}

uint8_t validateCaptureMode(uint8_t raw) {
  return isValidCaptureMode(raw) ? raw : DEFAULT_CAPTURE_MODE;
}

float decodeStoredGallons(uint16_t raw) {
  if (raw == STORED_GALLONS_UNPROGRAMMED) return 0.0f;
  return (float)raw / 1000.0f;
}

uint16_t encodeStoredGallons(float gallons) {
  float clamped = clampf(gallons, 0.0f, MAX_STORED_GALLONS);
  // Rounded rather than truncated: the nearest representable float to a value
  // like 65.534 is a shade under it, and truncating would lose a count. The
  // ceiling is chosen so this can never round up to the erased pattern.
  return (uint16_t)(clamped * 1000.0f + 0.5f);
}

uint16_t Settings::readU16(uint16_t addr) {
  return (uint16_t)(((uint16_t)store_.read(addr) << 8) |
                    (uint16_t)store_.read(addr + 1));
}

void Settings::writeU16(uint16_t addr, uint16_t value) {
  store_.update(addr, (uint8_t)(value >> 8));
  store_.update(addr + 1, (uint8_t)(value & 0xFF));
}

uint32_t Settings::readU32(uint16_t addr) {
  return ((uint32_t)store_.read(addr) << 24) |
         ((uint32_t)store_.read(addr + 1) << 16) |
         ((uint32_t)store_.read(addr + 2) << 8) |
         (uint32_t)store_.read(addr + 3);
}

void Settings::writeU32(uint16_t addr, uint32_t value) {
  store_.update(addr, (uint8_t)(value >> 24));
  store_.update(addr + 1, (uint8_t)((value >> 16) & 0xFF));
  store_.update(addr + 2, (uint8_t)((value >> 8) & 0xFF));
  store_.update(addr + 3, (uint8_t)(value & 0xFF));
}

uint8_t Settings::canSpeed() { return validateCanSpeed(store_.read(ADDR_CAN_SPEED)); }
void Settings::setCanSpeed(uint8_t value) {
  store_.update(ADDR_CAN_SPEED, validateCanSpeed(value));
}
bool Settings::canSpeedWasDefaulted() {
  return !isValidCanSpeed(store_.read(ADDR_CAN_SPEED));
}

uint8_t Settings::flowPin() { return validateFlowPin(store_.read(ADDR_FLOW_PIN)); }
void Settings::setFlowPin(uint8_t value) {
  store_.update(ADDR_FLOW_PIN, validateFlowPin(value));
}
bool Settings::flowPinWasDefaulted() {
  return !isValidFlowPin(store_.read(ADDR_FLOW_PIN));
}

bool Settings::metric() { return validateMetric(store_.read(ADDR_METRIC)); }
void Settings::setMetric(bool value) {
  store_.update(ADDR_METRIC, value ? 1 : 0);
}

uint32_t Settings::k() { return validateK(readU32(ADDR_K)); }
void Settings::setK(uint32_t value) { writeU32(ADDR_K, validateK(value)); }
bool Settings::kWasDefaulted() { return !isValidK(readU32(ADDR_K)); }

uint8_t Settings::captureMode() {
  return validateCaptureMode(store_.read(ADDR_CAPTURE_MODE));
}
void Settings::setCaptureMode(uint8_t value) {
  store_.update(ADDR_CAPTURE_MODE, validateCaptureMode(value));
}

// Set once the user has deliberately chosen a K, which is what stops the
// recalibration advice being offered forever - including to someone who has
// just followed it, since a correctly recalibrated K often lands outside the
// FT60 tolerance band.
bool Settings::kAcknowledged() {
  return store_.read(ADDR_K_ACKED) == K_ACKED_MAGIC;
}
void Settings::setKAcknowledged() {
  store_.update(ADDR_K_ACKED, K_ACKED_MAGIC);
}

// Anything that is not an explicit 0 or 1 - including the erased pattern -
// falls back to the default rather than being read as off.
bool validateCanResetEnabled(uint8_t raw) {
  if (raw == 0) return false;
  if (raw == 1) return true;
  return DEFAULT_CAN_RESET_ENABLED != 0;
}

bool Settings::canResetEnabled() {
  return validateCanResetEnabled(store_.read(ADDR_CAN_RESET));
}
void Settings::setCanResetEnabled(bool value) {
  store_.update(ADDR_CAN_RESET, value ? 1 : 0);
}

uint8_t Settings::lastResetReason() {
  uint8_t raw = store_.read(ADDR_LAST_RESET);
  return raw <= RESET_REASON_MAX ? raw : RESET_NONE;
}

uint8_t Settings::resetCount() {
  uint8_t raw = store_.read(ADDR_RESET_COUNT);
  return raw > MAX_RESET_COUNT ? 0 : raw;  // 0xFF means never written
}

void Settings::recordReset(uint8_t reason) {
  store_.update(ADDR_LAST_RESET,
                reason <= RESET_REASON_MAX ? reason : RESET_NONE);
  uint8_t n = resetCount();
  if (n < MAX_RESET_COUNT) store_.update(ADDR_RESET_COUNT, (uint8_t)(n + 1));
}

bool Settings::captureModeWasDefaulted() {
  return !isValidCaptureMode(store_.read(ADDR_CAPTURE_MODE));
}

uint8_t Settings::pulseGuardUnits() {
  return validatePulseGuard(store_.read(ADDR_PULSE_GUARD));
}
void Settings::setPulseGuardUnits(uint8_t value) {
  store_.update(ADDR_PULSE_GUARD, validatePulseGuard(value));
}
uint16_t Settings::pulseGuardUs() {
  return (uint16_t)((uint16_t)pulseGuardUnits() * PULSE_GUARD_UNIT_US);
}

float Settings::fuelUsedGallons() {
  return decodeStoredGallons(readU16(ADDR_FUEL_USED));
}
void Settings::setFuelUsedGallons(float gallons) {
  writeU16(ADDR_FUEL_USED, encodeStoredGallons(gallons));
}

}  // namespace fuel
