#include "Ring_EEPROM.h"
struct Payload { uint16_t x; uint8_t y[5]; };
void compile_api() {
  static const char sigText[] = "TALI";
  static const uint8_t sigBytes[] = {0xAA, 0x55, 0x11};
  Ring_EEPROM.init(AT24C64, sigText);
  Ring_EEPROM.init(AT24C64, sigBytes, 0, 8191);
  Ring_EEPROM.init(INTERNAL, 2222);
  Payload p = {};
  Ring_EEPROM.write(p);
  Ring_EEPROM.read(p);
}
