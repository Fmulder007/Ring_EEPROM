#include <Ring_EEPROM.h>
#include <Block_Ring_EEPROM.h>

struct CombinedPayload {
    uint16_t value;
    uint8_t flags;
};

void compile_both_libraries() {
    static const uint16_t signature = 0xA35C;
    CombinedPayload a = {};
    CombinedPayload b = {};

    (void)Ring_EEPROM.init(AT24C64, signature);
    (void)Ring_EEPROM.write(a);
    (void)Ring_EEPROM.read(a);

    (void)Block_Ring_EEPROM.init(BLOCK_AT24C64, signature);
    (void)Block_Ring_EEPROM.write(b);
    (void)Block_Ring_EEPROM.read(b);
}
