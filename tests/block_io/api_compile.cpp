#include "Block_Ring_EEPROM.h"
struct Payload { uint16_t x; uint8_t y[5]; };
void compile_api() {
  static const char sigText[] = "TALI";
  static const uint8_t sigBytes[] = {0xAA, 0x55, 0x11};
  Block_Ring_EEPROM.init(BLOCK_AT24C64, sigText);
  Block_Ring_EEPROM.init(BLOCK_AT24C64, sigBytes, 0, 8191);
  Block_Ring_EEPROM.init(BLOCK_INTERNAL, 2222);
  Payload p = {};
  Block_Ring_EEPROM.write(p);
  Block_Ring_EEPROM.read(p);
}
