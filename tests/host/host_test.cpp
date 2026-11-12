#include <stdint.h>
#include <string.h>
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

static uint8_t extMem[8192];
static uint8_t intMem[1024];
static uint8_t tx[40];
static uint8_t txLen;
static uint16_t memPtr;
static uint8_t rx[32];
static uint8_t rxLen, rxPos;
static bool devicePresent = true;
static bool corruptNextExternalWrite = false;
static uint32_t externalReadBytes = 0;
static bool nonByteI2CRead = false;
static bool nonByteI2CWrite = false;
EEPROMClass EEPROM;
TwoWire Wire;

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return HIGH; }
uint32_t micros(void) { static uint32_t x = 0; return ++x; }
uint32_t millis(void) { static uint32_t x = 0; return ++x; }
void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
uint8_t EEPROMClass::read(int a) const { return intMem[a]; }
void EEPROMClass::update(int a, uint8_t v) { intMem[a] = v; }
void TwoWire::begin() {}
void TwoWire::setClock(uint32_t) {}
void TwoWire::setWireTimeout(uint32_t, bool) {}
void TwoWire::clearWireTimeoutFlag() {}
bool TwoWire::getWireTimeoutFlag() { return false; }
void TwoWire::beginTransmission(uint8_t) { txLen = 0; }
size_t TwoWire::write(uint8_t v) { if (txLen < sizeof(tx)) tx[txLen++] = v; return 1; }
uint8_t TwoWire::endTransmission(bool) {
    if (!devicePresent) return 2;
    if (txLen >= 2) {
        memPtr = ((uint16_t)tx[0] << 8) | tx[1];
        if (txLen > 3) nonByteI2CWrite = true;
        if (txLen > 2) {
            for (uint8_t i = 2; i < txLen; ++i) {
                uint8_t value = tx[i];
                if (corruptNextExternalWrite && i == 2) {
                    value ^= 0x01;
                    corruptNextExternalWrite = false;
                }
                extMem[memPtr++] = value;
            }
        }
    }
    return 0;
}
uint8_t TwoWire::requestFrom(uint8_t, uint8_t n) {
    if (n != 1) nonByteI2CRead = true;
    externalReadBytes += n;
    rxLen = n;
    rxPos = 0;
    for (uint8_t i = 0; i < n; ++i) rx[i] = extMem[memPtr++];
    return n;
}
int TwoWire::available() { return rxLen - rxPos; }
int TwoWire::read() { return rxPos < rxLen ? rx[rxPos++] : -1; }

#include "Ring_EEPROM.h"

struct Tiny { uint8_t a; uint8_t b; uint16_t c; };
struct Large { uint8_t bytes[40]; };

static uint8_t crc8(const uint8_t *data, uint16_t length) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xE7) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint16_t crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8D95) : (uint16_t)(crc << 1);
    }
    return crc;
}

int main() {
    memset(extMem, 0xFF, sizeof(extMem));
    memset(intMem, 0xFF, sizeof(intMem));

    /* CRC reference vectors for the selected default polynomials. */
    const uint8_t check[] = "123456789";
    if (crc8(check, 9) != 0x34) return 1;
    if (crc16(check, 9) != 0x4FEF) return 2;

    /* External AT24C64, scalar/array signature, no record, then write/read. */
    const uint8_t signature[] = {0x52, 0x45, 0x31};
    uint8_t rc = Ring_EEPROM.init(AT24C64, signature);
    if (rc != RING_OK) return 3;
    Tiny out = {0, 0, 0};
    rc = Ring_EEPROM.read(out);
    if (rc != ERR_READ_NO_RECORD) return 4;

    Tiny tiny = {1, 2, 0x1234};
    rc = Ring_EEPROM.write(tiny);
    if (rc != RING_OK) return 5;
    externalReadBytes = 0;
    rc = Ring_EEPROM.read(out);
    if (rc != RING_OK || memcmp(&tiny, &out, sizeof(tiny)) != 0) return 6;
    if (externalReadBytes >= 128) return 20; // cache hit: без полного сканирования

    externalReadBytes = 0;
    rc = Ring_EEPROM.write(tiny);
    if (rc != RING_OK) return 21;
    if (externalReadBytes >= 128) return 22; // write() тоже использует кэш

    // Неудачная новая запись не заменяет последнюю успешно проверенную запись.
    corruptNextExternalWrite = true;
    rc = Ring_EEPROM.write(tiny);
    if (rc != ERR_WRITE_VERIFY) return 23;

    externalReadBytes = 0;
    rc = Ring_EEPROM.read(out);
    if (rc != RING_OK || memcmp(&tiny, &out, sizeof(tiny)) != 0) return 24;
    if (externalReadBytes >= 128) return 25; // старый last-good кэш сохранён

    /* Several sequential records. Latest is damaged: read() falls back one record. */
    for (uint8_t i = 0; i < 6; ++i) {
        tiny.a = i;
        tiny.b = (uint8_t)(i + 20);
        tiny.c = (uint16_t)(0x2000 + i);
        rc = Ring_EEPROM.write(tiny);
        if (rc != RING_OK) return 7;
    }
    /* B=2, CRC-8=1, L=10, P=11; newest start is 77 after the extra successful write. */
    extMem[77 + 9] ^= 0x80;
    rc = Ring_EEPROM.read(out);
    if (rc != RING_OK) return 8;
    if (out.a != 4) return 9;

    /* Damage the remaining three fallback candidates as well. */
    extMem[66 + 9] ^= 0x80;
    extMem[55 + 9] ^= 0x80;
    extMem[44 + 9] ^= 0x80;
    rc = Ring_EEPROM.read(out);
    if (rc != ERR_READ_CRC_FAILED) return 10;

    /* Write verification must return its dedicated code on read-back mismatch. */
    corruptNextExternalWrite = true;
    rc = Ring_EEPROM.write(tiny);
    if (rc != ERR_WRITE_VERIFY) return 11;

    /* CRC-16 path with a 40-byte object. */
    memset(extMem, 0xFF, sizeof(extMem));
    const char textSignature[] = "BIG";
    rc = Ring_EEPROM.init(AT24C64, textSignature);
    if (rc != RING_OK) return 12;
    Large large;
    for (uint8_t i = 0; i < sizeof(large.bytes); ++i) large.bytes[i] = (uint8_t)(i * 3U);
    rc = Ring_EEPROM.write(large);
    if (rc != RING_OK) return 13;
    Large largeOut;
    memset(&largeOut, 0, sizeof(largeOut));
    rc = Ring_EEPROM.read(largeOut);
    if (rc != RING_OK || memcmp(&large, &largeOut, sizeof(large)) != 0) return 14;

    /* Internal AVR EEPROM and explicitly bounded ring: write crosses ring end. */
    const uint8_t intSignature[] = {0x11, 0x22};
    rc = Ring_EEPROM.init(INTERNAL, intSignature, 100, 199);
    if (rc != RING_OK) return 15;
    memset(intMem + 100, 0xFF, 100);
    for (uint8_t i = 0; i < 12; ++i) {
        tiny.a = i;
        tiny.b = (uint8_t)(i ^ 0x5A);
        tiny.c = (uint16_t)(0x4000 + i);
        rc = Ring_EEPROM.write(tiny);
        if (rc != RING_OK) return 16;
    }
    memset(&out, 0, sizeof(out));
    rc = Ring_EEPROM.read(out);
    if (rc != RING_OK || out.a != 11) return 17;

    /* Out-of-range initialization is reported directly. */
    rc = Ring_EEPROM.init(INTERNAL, intSignature, 0, 1024);
    if (rc != ERR_ADDRESS_OUT_OF_RANGE) return 18;

    /* External chip absence is an init error, not a generic false. */
    devicePresent = false;
    rc = Ring_EEPROM.init(AT24C32, 0xA3);
    if (rc != ERR_I2C_NO_RESPONSE) return 19;

    // Внешняя EEPROM должна обслуживаться только однобайтовыми I2C-операциями.
    if (nonByteI2CRead || nonByteI2CWrite) return 26;

    return 0;
}
