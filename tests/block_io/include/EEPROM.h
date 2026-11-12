#ifndef TEST_EEPROM_H
#define TEST_EEPROM_H
#include <stdint.h>
class EEPROMClass {
public:
  uint8_t read(int address) const;
  void update(int address, uint8_t value);
};
extern EEPROMClass EEPROM;
#endif
