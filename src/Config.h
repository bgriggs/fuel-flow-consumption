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
const uint16_t ADDR_END = 10;       // first free byte

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
// CAPTURE_TIMER_POLL samples the pin from the 1 kHz TIMER0 compare interrupt.
// That cannot resolve a pulse train faster than ~500 Hz, and an FT60 at
// 68000 pulses/gal passes 500 Hz at roughly 26 GPH, so at racing flow rates it
// silently undercounts. It remains the default because existing units have
// their K-factor calibrated against it.
//
// CAPTURE_PIN_INTERRUPT counts every edge in hardware and is accurate at any
// flow rate the transducer supports, but K will need recalibrating after the
// switch. Requires a flow pin with an external interrupt.
const uint8_t CAPTURE_TIMER_POLL = 0;
const uint8_t CAPTURE_PIN_INTERRUPT = 1;
const uint8_t DEFAULT_CAPTURE_MODE = CAPTURE_TIMER_POLL;

bool isValidCanSpeed(uint8_t raw);
bool isValidFlowPin(uint8_t raw);
bool isValidMetric(uint8_t raw);
bool isValidK(uint32_t raw);
bool isValidCaptureMode(uint8_t raw);

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
