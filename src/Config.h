// Config.h - Persistent settings, with every stored value validated on the way
// out of EEPROM.
//
// An erased AVR EEPROM reads back as 0xFF. Used raw that gave a flow pin of
// 255 (out of range for pinMode/digitalRead), a K-factor of 4294967295, a CAN
// speed no driver accepts, and a "metric" flag that was true. Each getter here
// falls back to a documented default when the stored byte is not plausible, so
// a fresh or partially corrupted EEPROM still boots into a working device.
//
// The byte layout is unchanged from earlier firmware, so existing units keep
// their configured settings across the upgrade.
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

namespace fuel {

// Byte offsets. Do not renumber: these are live in deployed devices.
const uint16_t ADDR_FUEL_USED = 0;  // 2 bytes, gallons x1000, big-endian
const uint16_t ADDR_CAN_SPEED = 2;  // 1 byte, mcp_can speed enum
const uint16_t ADDR_FLOW_PIN = 3;   // 1 byte, digital pin number
const uint16_t ADDR_METRIC = 4;     // 1 byte, 0 or 1
const uint16_t ADDR_K = 5;          // 4 bytes, pulses per gallon, big-endian
const uint16_t ADDR_CAPTURE_MODE = 9;  // 1 byte, how pulses are counted
const uint16_t ADDR_PULSE_GUARD = 10;  // 1 byte, noise holdoff in 10 us units
const uint16_t ADDR_K_ACKED = 11;   // 1 byte, user has chosen a K deliberately
const uint16_t ADDR_END = 12;       // first free byte

// Arbitrary non-erased value, so an unprogrammed 0xFF does not read as "the
// user has acknowledged K".
const uint8_t K_ACKED_MAGIC = 0x5A;

// mcp_can speed enum range (CAN_5KBPS .. CAN_1000KBPS).
const uint8_t MIN_CAN_SPEED = 1;
const uint8_t MAX_CAN_SPEED = 18;
const uint8_t DEFAULT_CAN_SPEED = 16;  // CAN_500KBPS

// Highest digital pin on the ATmega32U4 core.
const uint8_t MAX_FLOW_PIN = 30;
const uint8_t DEFAULT_FLOW_PIN = 0;  // RX

// Plausible transducer K-factors, in pulses per gallon. The FT60 is ~68000.
const uint32_t MIN_K = 1000UL;
const uint32_t MAX_K = 1000000UL;
const uint32_t DEFAULT_K = 68000UL;

// gallons x1000 in a uint16 tops out at 65.535. Saved values are capped just
// below that so the all-ones pattern means "never written" and nothing else.
const uint16_t STORED_GALLONS_UNPROGRAMMED = 0xFFFF;
const float MAX_STORED_GALLONS = 65.534f;

// How the transducer output is counted.
//
// CAPTURE_PIN_INTERRUPT (the default) counts every falling edge in hardware
// and is accurate at any rate the transducer can produce.
//
// CAPTURE_TIMER_POLL samples the pin from the 1 kHz TIMER0 compare interrupt,
// which is what earlier firmware did. TIMER0 runs on a /64 prescaler, so the
// sample rate is 976.6 Hz and the shortest pulse it can resolve is 488 Hz. At
// the FT60's 68000 pulses/gallon that is only 25.9 GPH - well inside the
// transducer's 0.6 to 70 GPH range - and the failure is not graceful: around
// 977 Hz (51.7 GPH) the samples land on the same phase every time and the
// count collapses toward zero. Kept only as a fallback for a flow pin with no
// external interrupt.
const uint8_t CAPTURE_TIMER_POLL = 0;
const uint8_t CAPTURE_PIN_INTERRUPT = 1;
const uint8_t DEFAULT_CAPTURE_MODE = CAPTURE_PIN_INTERRUPT;

bool isValidCanSpeed(uint8_t raw);
bool isValidFlowPin(uint8_t raw);
bool isValidMetric(uint8_t raw);
bool isValidK(uint32_t raw);
bool isValidCaptureMode(uint8_t raw);

// Minimum spacing between accepted pulses in interrupt capture, stored in
// units of 10 us to fit a byte.
//
// This is the only noise control the firmware has. Polling was an accidental
// 977 Hz low-pass filter; counting edges in hardware removes that, so a burst
// of ringing or a coupled spike train would otherwise each be counted. The
// holdoff caps the countable rate at 1/guard.
//
// It has to stay below the shortest real pulse period or it starts discarding
// fuel. An FT60 at 68000 pulses/gallon and its rated 70 GPH runs at 1322 Hz,
// a 756 us period, so the 400 us default still passes 2500 Hz (132 GPH at
// that K) while rejecting everything faster. A transducer with a much higher
// K needs a shorter guard, which is why 'status' reports the countable
// ceiling in GPH for the K in use.
const uint16_t PULSE_GUARD_UNIT_US = 10;
const uint8_t MAX_PULSE_GUARD_UNITS = 200;      // 2000 us
const uint8_t DEFAULT_PULSE_GUARD_UNITS = 40;   // 400 us
bool isValidPulseGuard(uint8_t raw);
uint8_t validatePulseGuard(uint8_t raw);

// Highest pulse rate still counted with the given holdoff, in Hz. Zero guard
// means no limit, reported as 0.
uint16_t countableRateHz(uint16_t guardUs);

// The FT60 is specified at 68000 pulses/gallon, and its datasheet notes that
// installation can shift that somewhat. A K a long way below nominal is the
// signature of a value dialed down to compensate for a counter that was
// losing pulses, and it will over-read once every pulse is counted.
const uint32_t K_TOLERANCE_PERCENT = 10;
bool isNearNominalK(uint32_t k);

uint8_t validateCanSpeed(uint8_t raw);
uint8_t validateFlowPin(uint8_t raw);
bool validateMetric(uint8_t raw);
uint32_t validateK(uint32_t raw);
uint8_t validateCaptureMode(uint8_t raw);

// Fuel-used persistence. An unprogrammed or implausible cell decodes to 0
// rather than the 65.535 gallons that 0xFFFF literally represents.
float decodeStoredGallons(uint16_t raw);
uint16_t encodeStoredGallons(float gallons);

// Byte-level EEPROM access, abstracted so the settings logic can be driven by
// a fake in the tests. No virtual destructor: instances are never destroyed
// through this pointer, and declaring one would pull operator delete into the
// AVR image.
class EepromStore {
public:
  virtual uint8_t read(uint16_t addr) = 0;
  virtual void update(uint16_t addr, uint8_t value) = 0;
protected:
  ~EepromStore() {}
};

class Settings {
public:
  explicit Settings(EepromStore& store) : store_(store) {}

  uint8_t canSpeed();
  void setCanSpeed(uint8_t value);

  uint8_t flowPin();
  void setFlowPin(uint8_t value);

  bool metric();
  void setMetric(bool value);

  uint32_t k();
  void setK(uint32_t value);

  uint8_t captureMode();
  void setCaptureMode(uint8_t value);
  // True while the stored byte has never been written. Used to tell an
  // upgrade from a deliberate choice.
  bool captureModeWasDefaulted();

  bool kAcknowledged();
  void setKAcknowledged();

  uint8_t pulseGuardUnits();
  void setPulseGuardUnits(uint8_t value);
  uint16_t pulseGuardUs();

  float fuelUsedGallons();
  void setFuelUsedGallons(float gallons);

  // True when the stored byte(s) failed validation and a default was
  // substituted, so the sketch can say so over the serial console.
  bool canSpeedWasDefaulted();
  bool flowPinWasDefaulted();
  bool kWasDefaulted();

private:
  uint16_t readU16(uint16_t addr);
  void writeU16(uint16_t addr, uint16_t value);
  uint32_t readU32(uint16_t addr);
  void writeU32(uint16_t addr, uint32_t value);

  EepromStore& store_;
};

}  // namespace fuel

#endif  // CONFIG_H
