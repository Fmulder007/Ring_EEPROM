#include <Block_Ring_EEPROM.h>

struct UserData {
    uint8_t mode;
    uint16_t frequency;
    float swr;
    char name[20];
};

/*
  Числовая сигнатура обычно занимает 2 байта на AVR.
  Можно также передать строку или массив байтов.
*/
const uint16_t DATA_SIGNATURE = 0xA35C;

UserData data = {1, 7100000, 1.25f, "KARAT-3.5"};
UserData restored = {};

void setup() {
    Serial.begin(115200);

    /* Выбери нужную память:
       BLOCK_INTERNAL — внутренняя EEPROM AVR;
       BLOCK_AT24C32  — внешняя EEPROM 4096 байт;
       BLOCK_AT24C64  — внешняя EEPROM 8192 байта.
    */
    uint8_t result = Block_Ring_EEPROM.init(BLOCK_INTERNAL, DATA_SIGNATURE);

    if (result != BLOCK_RING_OK) {
        Serial.print(F("init error: "));
        Serial.println(result);
        return;
    }

    result = Block_Ring_EEPROM.write(data);
    if (result != BLOCK_RING_OK) {
        Serial.print(F("write error: "));
        Serial.println(result);
        return;
    }

    result = Block_Ring_EEPROM.read(restored);
    if (result == BLOCK_RING_OK) {
        Serial.println(F("Read OK"));
        Serial.print(F("Frequency: "));
        Serial.println(restored.frequency);
        Serial.print(F("Name: "));
        Serial.println(restored.name);
    } else {
        Serial.print(F("read error: "));
        Serial.println(result);
    }
}

void loop() {
}
