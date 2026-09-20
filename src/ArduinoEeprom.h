// ArduinoEeprom.h - Binds the EepromStore interface to the AVR EEPROM.
//
// Only ever included from the .ino files. Keeping it out of the .cpp sources
// is what lets the settings logic be tested on a host against a fake store.
#ifndef ARDUINO_EEPROM_H
#define ARDUINO_EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>

#include "Config.h"

namespace fuel {

class ArduinoEeprom : public EepromStore {
public:
  uint8_t read(uint16_t addr) { return EEPROM.read((int)addr); }

  // EEPROM.update() skips the write when the byte is unchanged, which matters
  // because the fuel-used cell is rewritten every 30 seconds against a
  // 100,000 cycle endurance rating.
  void update(uint16_t addr, uint8_t value) { EEPROM.update((int)addr, value); }
};

}  // namespace fuel

#endif  // ARDUINO_EEPROM_H
