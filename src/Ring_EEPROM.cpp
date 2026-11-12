#include "Ring_EEPROM.h"

#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
#include <EEPROM.h>
#endif

#if (RING_EEPROM_SDA == A4) && (RING_EEPROM_SCL == A5)
#define RING_EEPROM_USE_HARDWARE_I2C 1
#include <Wire.h>
#else
#define RING_EEPROM_USE_HARDWARE_I2C 0
#endif

#if RING_EEPROM_FALLBACK_DEPTH < 0
#error "RING_EEPROM_FALLBACK_DEPTH must not be negative"
#endif
#if RING_EEPROM_FALLBACK_DEPTH > 32
#error "RING_EEPROM_FALLBACK_DEPTH is limited to 32 in this implementation"
#endif
#if RING_EEPROM_INLINE_SIGNATURE_MAX < 1
#error "RING_EEPROM_INLINE_SIGNATURE_MAX must be at least 1"
#endif
#if RING_EEPROM_IO_BUFFER_SIZE < 4
#error "RING_EEPROM_IO_BUFFER_SIZE must be at least 4"
#endif

RingEEPROM Ring_EEPROM;

namespace {

#if RING_EEPROM_USE_HARDWARE_I2C
static uint8_t mapWireError(uint8_t status) {
    if (status == 0) return RING_OK;
    if (status == 5) return ERR_I2C_TIMEOUT;
    if (status == 2 || status == 3) return ERR_I2C_NO_RESPONSE;
    return ERR_I2C_INIT_FAILED;
}
#endif

#if !RING_EEPROM_USE_HARDWARE_I2C
static void softDelay() {
    delayMicroseconds(RING_EEPROM_SOFT_I2C_DELAY_US);
}

static void softRelease(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

static void softLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

static bool softWaitClockHigh() {
    const uint32_t started = micros();
    softRelease(RING_EEPROM_SCL);
    while (digitalRead(RING_EEPROM_SCL) == LOW) {
        if ((uint32_t)(micros() - started) >= RING_EEPROM_SOFT_I2C_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

static bool softStart() {
    softRelease(RING_EEPROM_SDA);
    if (!softWaitClockHigh()) return false;
    softDelay();
    if (digitalRead(RING_EEPROM_SDA) == LOW) return false;
    softLow(RING_EEPROM_SDA);
    softDelay();
    softLow(RING_EEPROM_SCL);
    softDelay();
    return true;
}

static bool softStop() {
    softLow(RING_EEPROM_SDA);
    softDelay();
    if (!softWaitClockHigh()) {
        softRelease(RING_EEPROM_SDA);
        return false;
    }
    softDelay();
    softRelease(RING_EEPROM_SDA);
    softDelay();
    return digitalRead(RING_EEPROM_SDA) == HIGH;
}

static bool softWriteByte(uint8_t value) {
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
        if (value & mask) softRelease(RING_EEPROM_SDA);
        else softLow(RING_EEPROM_SDA);
        softDelay();
        if (!softWaitClockHigh()) return false;
        softDelay();
        softLow(RING_EEPROM_SCL);
    }

    softRelease(RING_EEPROM_SDA);
    softDelay();
    if (!softWaitClockHigh()) return false;
    softDelay();
    const bool acknowledged = (digitalRead(RING_EEPROM_SDA) == LOW);
    softLow(RING_EEPROM_SCL);
    softDelay();
    return acknowledged;
}

static bool softReadByte(uint8_t &value, bool acknowledge) {
    value = 0;
    softRelease(RING_EEPROM_SDA);
    for (uint8_t i = 0; i < 8; ++i) {
        value <<= 1;
        if (!softWaitClockHigh()) return false;
        softDelay();
        if (digitalRead(RING_EEPROM_SDA) == HIGH) value |= 1;
        softLow(RING_EEPROM_SCL);
        softDelay();
    }

    if (acknowledge) softLow(RING_EEPROM_SDA);
    else softRelease(RING_EEPROM_SDA);
    softDelay();
    if (!softWaitClockHigh()) return false;
    softDelay();
    softLow(RING_EEPROM_SCL);
    softRelease(RING_EEPROM_SDA);
    softDelay();
    return true;
}

static uint8_t softProbe() {
    if (!softStart()) return ERR_I2C_TIMEOUT;
    const bool ack = softWriteByte((uint8_t)(RING_EEPROM_I2C_ADDRESS << 1));
    const bool stopped = softStop();
    if (!stopped) return ERR_I2C_TIMEOUT;
    return ack ? RING_OK : ERR_I2C_NO_RESPONSE;
}

static uint8_t softReadMemory(uint16_t address, uint8_t *destination, uint16_t length) {
    if (!softStart()) return ERR_I2C_TIMEOUT;
    if (!softWriteByte((uint8_t)(RING_EEPROM_I2C_ADDRESS << 1)) ||
        !softWriteByte((uint8_t)(address >> 8)) ||
        !softWriteByte((uint8_t)address)) {
        softStop();
        return ERR_I2C_NO_RESPONSE;
    }
    if (!softStart()) {
        softStop();
        return ERR_I2C_TIMEOUT;
    }
    if (!softWriteByte((uint8_t)((RING_EEPROM_I2C_ADDRESS << 1) | 1))) {
        softStop();
        return ERR_I2C_NO_RESPONSE;
    }
    for (uint16_t i = 0; i < length; ++i) {
        if (!softReadByte(destination[i], i + 1 < length)) {
            softStop();
            return ERR_I2C_TIMEOUT;
        }
    }
    return softStop() ? RING_OK : ERR_I2C_TIMEOUT;
}

static uint8_t softWriteMemory(uint16_t address, const uint8_t *source, uint16_t length) {
    if (!softStart()) return ERR_I2C_TIMEOUT;
    if (!softWriteByte((uint8_t)(RING_EEPROM_I2C_ADDRESS << 1)) ||
        !softWriteByte((uint8_t)(address >> 8)) ||
        !softWriteByte((uint8_t)address)) {
        softStop();
        return ERR_I2C_NO_RESPONSE;
    }
    for (uint16_t i = 0; i < length; ++i) {
        if (!softWriteByte(source[i])) {
            softStop();
            return ERR_I2C_NO_RESPONSE;
        }
    }
    if (!softStop()) return ERR_I2C_TIMEOUT;

    const uint32_t started = millis();
    for (;;) {
        if (softStart()) {
            const bool ack = softWriteByte((uint8_t)(RING_EEPROM_I2C_ADDRESS << 1));
            softStop();
            if (ack) return RING_OK;
        }
        if ((uint32_t)(millis() - started) >= RING_EEPROM_I2C_TIMEOUT_MS) {
            return ERR_I2C_TIMEOUT;
        }
        delay(1);
    }
}
#endif

} // namespace

RingEEPROM::RingEEPROM()
    : _device(0xFF),
      _initialized(0),
      _softI2C(0),
      _capacity(0),
      _start(0),
      _end(0),
      _ringSize(0),
      _signature(0),
      _signatureLength(0),
      _sequenceBytes(0),
      _crcBytes(0),
      _recordLength(0),
      _step(0),
      _dataLength(0),
      _hasLastGood(false),
      _lastGoodOffset(0),
      _lastGoodSequence(0) {
    memset(_signatureInline, 0, sizeof(_signatureInline));
    memset(_ioBuffer, 0, sizeof(_ioBuffer));
}

uint8_t RingEEPROM::init(uint8_t device, const uint8_t *signature, uint16_t signatureLength) {
    return initArray(device, signature, signatureLength, false, 0, 0);
}

uint8_t RingEEPROM::init(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                         uint16_t startAddress, uint16_t endAddress) {
    return initArray(device, signature, signatureLength, true, startAddress, endAddress);
}

uint8_t RingEEPROM::initScalar(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                               bool hasRange, uint16_t startAddress, uint16_t endAddress) {
    return initCommon(device, signature, signatureLength, true, hasRange, startAddress, endAddress);
}

uint8_t RingEEPROM::initArray(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                              bool hasRange, uint16_t startAddress, uint16_t endAddress) {
    /* Короткие массивы копируются, чтобы локальная переменная не потеряла срок жизни.
       Длинная подпись хранится по указателю; её исходный массив должен оставаться доступен. */
    const bool copySignature = (signatureLength <= RING_EEPROM_INLINE_SIGNATURE_MAX);
    return initCommon(device, signature, signatureLength, copySignature, hasRange,
                      startAddress, endAddress);
}

uint8_t RingEEPROM::initCommon(uint8_t device, const uint8_t *signature,
                               uint16_t signatureLength, bool copySignature,
                               bool hasRange, uint16_t startAddress, uint16_t endAddress) {
    _initialized = 0;
    /* Arduino AVR already uses INTERNAL for analogReference(); accept that value too. */
    if (device == (uint8_t)INTERNAL || device == (uint8_t)RING_EEPROM_INTERNAL) {
        device = (uint8_t)RING_EEPROM_INTERNAL;
    }
    _device = device;
    _capacity = 0;
    _ringSize = 0;
    _signature = 0;
    _signatureLength = 0;

    if (signature == 0 || signatureLength == 0) return ERR_SIGNATURE_INVALID;
    if (device != (uint8_t)RING_EEPROM_INTERNAL &&
        device != (uint8_t)AT24C32 &&
        device != (uint8_t)AT24C64) return ERR_UNKNOWN_DEVICE;

    if (copySignature) {
        if (signatureLength > sizeof(_signatureInline)) return ERR_SIGNATURE_TOO_LONG;
        memcpy(_signatureInline, signature, signatureLength);
        _signature = _signatureInline;
    } else {
        _signature = signature;
    }
    _signatureLength = signatureLength;

    if (device == RING_EEPROM_INTERNAL) {
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
    #if defined(E2END)
        _capacity = (uint16_t)(E2END + 1UL);
    #else
        return ERR_INTERNAL_EEPROM_UNAVAILABLE;
    #endif
#else
        return ERR_DEVICE_NOT_SUPPORTED;
#endif
    } else if (device == AT24C32) {
        _capacity = (uint16_t)RING_EEPROM_AT24C32_SIZE;
    } else if (device == AT24C64) {
        _capacity = (uint16_t)RING_EEPROM_AT24C64_SIZE;
    }

    if (_capacity == 0) return ERR_INTERNAL_EEPROM_UNAVAILABLE;

    if (hasRange) {
        if (startAddress > endAddress) return ERR_INVALID_RANGE;
        if (startAddress >= _capacity || endAddress >= _capacity) return ERR_ADDRESS_OUT_OF_RANGE;
        _start = startAddress;
        _end = endAddress;
    } else {
        _start = 0;
        _end = (uint16_t)(_capacity - 1);
    }
    _ringSize = (uint16_t)(_end - _start + 1U);
    if (_ringSize == 0) return ERR_INVALID_RANGE;

    if (device != RING_EEPROM_INTERNAL) {
#if RING_EEPROM_USE_HARDWARE_I2C
        _softI2C = 0;
        Wire.begin();
        Wire.setClock(RING_EEPROM_I2C_CLOCK);
        Wire.setWireTimeout((uint32_t)RING_EEPROM_I2C_TIMEOUT_MS * 1000UL, true);
        Wire.beginTransmission((uint8_t)RING_EEPROM_I2C_ADDRESS);
        const uint8_t wireStatus = Wire.endTransmission();
        if (wireStatus != 0) return mapWireError(wireStatus);
#else
        _softI2C = 1;
        softRelease(RING_EEPROM_SDA);
        softRelease(RING_EEPROM_SCL);
        delayMicroseconds(10);
        const uint8_t probeStatus = softProbe();
        if (probeStatus != RING_OK) return probeStatus;
#endif
    }

    _initialized = 1;
    _hasLastGood = false;
    return RING_OK;
}

uint8_t RingEEPROM::selectLayout(uint16_t dataLength) {
    if (!_initialized) return ERR_NOT_INITIALIZED;
    if (dataLength == 0) return ERR_INVALID_ARGUMENT;

    for (uint8_t sequenceBytes = 1; sequenceBytes <= 2; ++sequenceBytes) {
        const uint32_t protectedLength = (uint32_t)_signatureLength + sequenceBytes + dataLength;
        const uint8_t crcBytes = (protectedLength <= RING_EEPROM_CRC8_MAX_BYTES) ? 1 : 2;
        const uint32_t recordLength = protectedLength + crcBytes;
        if (recordLength > _ringSize || recordLength > 65535UL) return ERR_NO_SPACE;

        const uint32_t possibleRecords = _ringSize / recordLength;
        const uint32_t sequenceRange = (sequenceBytes == 1) ? 256UL : 65536UL;
        if (sequenceRange > (2UL * possibleRecords)) {
            _sequenceBytes = sequenceBytes;
            _crcBytes = crcBytes;
            _recordLength = (uint16_t)recordLength;
            _dataLength = dataLength;

            uint32_t step = _recordLength;
            uint16_t attempts = 0;
            while (gcd(step, _ringSize) != 1UL && attempts <= _ringSize) {
                ++step;
                ++attempts;
            }
            if (gcd(step, _ringSize) != 1UL) return ERR_INVALID_ARGUMENT;
            _step = (uint16_t)(step % _ringSize);
            return RING_OK;
        }
    }
    return ERR_NO_SPACE;
}

uint32_t RingEEPROM::gcd(uint32_t a, uint32_t b) const {
    while (b != 0) {
        const uint32_t r = a % b;
        a = b;
        b = r;
    }
    return a;
}

bool RingEEPROM::isNewer(uint16_t a, uint16_t b) const {
    const uint32_t range = (_sequenceBytes == 1) ? 256UL : 65536UL;
    const uint32_t mask = range - 1UL;
    const uint32_t difference = ((uint32_t)a - (uint32_t)b) & mask;
    return difference != 0 && difference < (range / 2UL);
}

void RingEEPROM::insertCandidate(Candidate *candidates, uint8_t &count,
                                 uint16_t offset, uint16_t sequence) {
    const uint8_t maximum = (uint8_t)(RING_EEPROM_FALLBACK_DEPTH + 1);
    for (uint8_t i = 0; i < count; ++i) {
        if (candidates[i].offset == offset) return;
    }

    uint8_t position = 0;
    while (position < count && !isNewer(sequence, candidates[position].sequence)) {
        ++position;
    }
    if (position >= maximum) return;

    if (count < maximum) ++count;
    for (uint8_t i = (uint8_t)(count - 1); i > position; --i) {
        candidates[i] = candidates[i - 1];
    }
    candidates[position].offset = offset;
    candidates[position].sequence = sequence;
}

uint8_t RingEEPROM::readSequence(uint16_t offset, uint16_t &sequence) {
    uint8_t bytes[2] = {0, 0};
    const uint8_t status = readLogical((uint16_t)(offset + _signatureLength), bytes, _sequenceBytes);
    if (status != RING_OK) return status;
    sequence = bytes[0];
    if (_sequenceBytes == 2) sequence |= (uint16_t)bytes[1] << 8;
    return RING_OK;
}

uint8_t RingEEPROM::scanSignature(uint16_t offset, bool &matches) {
    matches = false;
    uint16_t done = 0;
    while (done < _signatureLength) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(_signatureLength - done) > (uint32_t)sizeof(_ioBuffer))
                                      ? sizeof(_ioBuffer) : (_signatureLength - done));
        const uint8_t status = readLogical((uint16_t)(offset + done), _ioBuffer, chunk);
        if (status != RING_OK) return status;
        if (memcmp(_ioBuffer, _signature + done, chunk) != 0) return RING_OK;
        done += chunk;
    }
    matches = true;
    return RING_OK;
}

uint8_t RingEEPROM::findCandidates(Candidate *candidates, uint8_t &count, bool &signatureSeen) {
    count = 0;
    signatureSeen = false;
    const uint8_t first = _signature[0];
    uint16_t offset = 0;

    /* Отдельный блок сохраняет данные сканирования: scanSignature() использует _ioBuffer. */
    uint8_t scanBuffer[RING_EEPROM_IO_BUFFER_SIZE];
    while (offset < _ringSize) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(_ringSize - offset) > (uint32_t)sizeof(scanBuffer))
                                      ? sizeof(scanBuffer) : (_ringSize - offset));
        const uint8_t status = readLogical(offset, scanBuffer, chunk);
        if (status != RING_OK) return status;

        for (uint16_t i = 0; i < chunk; ++i) {
            if (scanBuffer[i] != first) continue;
            const uint16_t candidateOffset = (uint16_t)(offset + i);
            bool matches = false;
            const uint8_t compareStatus = scanSignature(candidateOffset, matches);
            if (compareStatus != RING_OK) return compareStatus;
            if (!matches) continue;

            signatureSeen = true;
            uint16_t sequence = 0;
            const uint8_t sequenceStatus = readSequence(candidateOffset, sequence);
            if (sequenceStatus != RING_OK) return sequenceStatus;
            insertCandidate(candidates, count, candidateOffset, sequence);
        }
        offset = (uint16_t)(offset + chunk);
    }
    return RING_OK;
}

uint8_t RingEEPROM::crcUpdate(uint8_t crc, uint8_t value) const {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ (uint8_t)RING_EEPROM_CRC8_POLY)
                            : (uint8_t)(crc << 1);
    }
    return crc;
}

uint16_t RingEEPROM::crcUpdate(uint16_t crc, uint8_t value) const {
    crc ^= (uint16_t)value << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ (uint16_t)RING_EEPROM_CRC16_POLY)
                             : (uint16_t)(crc << 1);
    }
    return crc;
}

uint8_t RingEEPROM::verifyRecord(uint16_t startOffset, uint16_t expectedSequence,
                                 uint8_t *destinationOrNull) {
    uint8_t crc8 = 0;
    uint16_t crc16 = 0;
    uint16_t logical = startOffset;
    uint16_t done = 0;

    /* Сигнатура входит в CRC и одновременно перепроверяется с памятью. */
    while (done < _signatureLength) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(_signatureLength - done) > (uint32_t)sizeof(_ioBuffer))
                                      ? sizeof(_ioBuffer) : (_signatureLength - done));
        uint8_t status = readLogical(logical, _ioBuffer, chunk);
        if (status != RING_OK) return status;
        if (memcmp(_ioBuffer, _signature + done, chunk) != 0) return ERR_SIGNATURE_INVALID;
        for (uint16_t i = 0; i < chunk; ++i) {
            if (_crcBytes == 1) crc8 = crcUpdate(crc8, _ioBuffer[i]);
            else crc16 = crcUpdate(crc16, _ioBuffer[i]);
        }
        logical = (uint16_t)(logical + chunk);
        done += chunk;
    }

    uint8_t sequenceData[2] = {0, 0};
    uint8_t status = readLogical(logical, sequenceData, _sequenceBytes);
    if (status != RING_OK) return status;
    uint16_t actualSequence = sequenceData[0];
    if (_sequenceBytes == 2) actualSequence |= (uint16_t)sequenceData[1] << 8;
    if (actualSequence != expectedSequence) return ERR_SIGNATURE_INVALID;
    for (uint8_t i = 0; i < _sequenceBytes; ++i) {
        if (_crcBytes == 1) crc8 = crcUpdate(crc8, sequenceData[i]);
        else crc16 = crcUpdate(crc16, sequenceData[i]);
    }
    logical = (uint16_t)(logical + _sequenceBytes);

    done = 0;
    while (done < _dataLength) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(_dataLength - done) > (uint32_t)sizeof(_ioBuffer))
                                      ? sizeof(_ioBuffer) : (_dataLength - done));
        status = readLogical((uint16_t)(logical + done), _ioBuffer, chunk);
        if (status != RING_OK) return status;
        for (uint16_t i = 0; i < chunk; ++i) {
            if (_crcBytes == 1) crc8 = crcUpdate(crc8, _ioBuffer[i]);
            else crc16 = crcUpdate(crc16, _ioBuffer[i]);
        }
        done += chunk;
    }
    logical = (uint16_t)(logical + _dataLength);

    uint8_t storedCrc[2] = {0, 0};
    status = readLogical(logical, storedCrc, _crcBytes);
    if (status != RING_OK) return status;
    if (_crcBytes == 1) {
        if (storedCrc[0] != crc8) return ERR_READ_CRC_FAILED;
    } else {
        const uint16_t stored = (uint16_t)storedCrc[0] | ((uint16_t)storedCrc[1] << 8);
        if (stored != crc16) return ERR_READ_CRC_FAILED;
    }

    /* Данные копируются только после успешного CRC и только по запросу read(). */
    if (destinationOrNull != 0) {
        const uint16_t dataOffset = (uint16_t)(startOffset + _signatureLength + _sequenceBytes);
        status = readLogical(dataOffset, destinationOrNull, _dataLength);
        if (status != RING_OK) return status;
    }
    return RING_OK;
}

uint8_t RingEEPROM::readRaw(uint8_t *destination, uint16_t dataLength) {
    if (!_initialized) return ERR_NOT_INITIALIZED;
    if (destination == 0) return ERR_INVALID_ARGUMENT;

    uint8_t status = selectLayout(dataLength);
    if (status != RING_OK) return status;

    // Быстрый путь: проверяем последнюю успешно сохранённую позицию.
    if (_hasLastGood) {
        status = verifyRecord(_lastGoodOffset, _lastGoodSequence, destination);
        if (status == RING_OK) return RING_OK;

        // Если запись повреждена, сбрасываем кэш и выполняем полный поиск.
        // Ошибки физического чтения возвращаем вызывающему коду без подмены.
        if (status != ERR_READ_CRC_FAILED && status != ERR_SIGNATURE_INVALID) {
            return status;
        }
        _hasLastGood = false;
    }

    // Полный поиск используется при пустом/сброшенном кэше или его повреждении.
    Candidate candidates[RING_EEPROM_FALLBACK_DEPTH + 1];
    uint8_t count = 0;
    bool signatureSeen = false;
    status = findCandidates(candidates, count, signatureSeen);
    if (status != RING_OK) return status;
    if (!signatureSeen || count == 0) return ERR_READ_NO_RECORD;

    for (uint8_t i = 0; i < count; ++i) {
        status = verifyRecord(candidates[i].offset, candidates[i].sequence, destination);
        if (status == RING_OK) {
            _hasLastGood = true;
            _lastGoodOffset = candidates[i].offset;
            _lastGoodSequence = candidates[i].sequence;
            return RING_OK;
        }
        if (status != ERR_READ_CRC_FAILED && status != ERR_SIGNATURE_INVALID) {
            return status;
        }
    }
    return ERR_READ_CRC_FAILED;
}

uint8_t RingEEPROM::writeRaw(const uint8_t *source, uint16_t dataLength) {
    if (!_initialized) return ERR_NOT_INITIALIZED;
    if (source == 0) return ERR_INVALID_ARGUMENT;

    uint8_t status = selectLayout(dataLength);
    if (status != RING_OK) return status;

    bool havePrevious = false;
    uint16_t previousOffset = 0;
    uint16_t previousSequence = 0;

    // Быстрый путь: используем кэш как предыдущую успешную запись.
    if (_hasLastGood) {
        status = verifyRecord(_lastGoodOffset, _lastGoodSequence, 0);
        if (status == RING_OK) {
            havePrevious = true;
            previousOffset = _lastGoodOffset;
            previousSequence = _lastGoodSequence;
        } else if (status == ERR_READ_CRC_FAILED || status == ERR_SIGNATURE_INVALID) {
            _hasLastGood = false;
        } else {
            // При ошибке физического чтения запись не считается повреждённой.
            return status;
        }
    }

    // Если кэш отсутствует или оказался повреждён, ищем предыдущую запись обычным способом.
    if (!havePrevious) {
        Candidate candidates[RING_EEPROM_FALLBACK_DEPTH + 1];
        uint8_t count = 0;
        bool signatureSeen = false;
        status = findCandidates(candidates, count, signatureSeen);
        if (status != RING_OK) return status;

        for (uint8_t i = 0; i < count; ++i) {
            status = verifyRecord(candidates[i].offset, candidates[i].sequence, 0);
            if (status == RING_OK) {
                havePrevious = true;
                previousOffset = candidates[i].offset;
                previousSequence = candidates[i].sequence;
                break;
            }
            if (status != ERR_READ_CRC_FAILED && status != ERR_SIGNATURE_INVALID) {
                return status;
            }
        }
    }

    // Вычисляем следующий адрес и номер записи.
    uint16_t nextOffset = 0;
    uint16_t nextSequence = 0;
    if (havePrevious) {
        nextOffset = (uint16_t)(((uint32_t)previousOffset + _step) % _ringSize);
        const uint16_t mask = (_sequenceBytes == 1) ? 0x00FFU : 0xFFFFU;
        nextSequence = (uint16_t)((previousSequence + 1U) & mask);
    }

    status = writeRecord(nextOffset, nextSequence, source);
    if (status != RING_OK) return status;

    // Проверяем новую запись чтением обратно. При неудаче старый кэш не заменяем:
    // последней успешной записью остаётся предыдущая проверенная запись.
    status = verifyRecord(nextOffset, nextSequence, 0);
    if (status == ERR_READ_CRC_FAILED || status == ERR_SIGNATURE_INVALID) {
        return ERR_WRITE_VERIFY;
    }
    if (status != RING_OK) return status;

    // Кэш обновляется только после успешной проверки новой записи.
    _hasLastGood = true;
    _lastGoodOffset = nextOffset;
    _lastGoodSequence = nextSequence;
    return RING_OK;
}

uint8_t RingEEPROM::writeRecord(uint16_t startOffset, uint16_t sequence, const uint8_t *data) {
    uint8_t status;
    uint8_t crc8 = 0;
    uint16_t crc16 = 0;
    uint16_t logical = startOffset;

    status = writeLogical(logical, _signature, _signatureLength);
    if (status != RING_OK) return status;
    for (uint16_t i = 0; i < _signatureLength; ++i) {
        if (_crcBytes == 1) crc8 = crcUpdate(crc8, _signature[i]);
        else crc16 = crcUpdate(crc16, _signature[i]);
    }
    logical = (uint16_t)(logical + _signatureLength);

    uint8_t sequenceData[2];
    sequenceData[0] = (uint8_t)sequence;
    sequenceData[1] = (uint8_t)(sequence >> 8);
    status = writeLogical(logical, sequenceData, _sequenceBytes);
    if (status != RING_OK) return status;
    for (uint8_t i = 0; i < _sequenceBytes; ++i) {
        if (_crcBytes == 1) crc8 = crcUpdate(crc8, sequenceData[i]);
        else crc16 = crcUpdate(crc16, sequenceData[i]);
    }
    logical = (uint16_t)(logical + _sequenceBytes);

    uint16_t done = 0;
    while (done < _dataLength) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(_dataLength - done) > (uint32_t)sizeof(_ioBuffer))
                                      ? sizeof(_ioBuffer) : (_dataLength - done));
        for (uint16_t i = 0; i < chunk; ++i) {
            if (_crcBytes == 1) crc8 = crcUpdate(crc8, data[done + i]);
            else crc16 = crcUpdate(crc16, data[done + i]);
        }
        status = writeLogical((uint16_t)(logical + done), data + done, chunk);
        if (status != RING_OK) return status;
        done += chunk;
    }
    logical = (uint16_t)(logical + _dataLength);

    uint8_t crcData[2];
    crcData[0] = (_crcBytes == 1) ? crc8 : (uint8_t)crc16;
    crcData[1] = (uint8_t)(crc16 >> 8);
    return writeLogical(logical, crcData, _crcBytes);
}

uint8_t RingEEPROM::readLogical(uint16_t offset, uint8_t *destination, uint16_t length) {
    if (_ringSize == 0 || destination == 0) return ERR_INVALID_ARGUMENT;
    uint32_t logical = offset % _ringSize;
    uint16_t done = 0;
    while (done < length) {
        const uint16_t contiguous = (uint16_t)(_ringSize - logical);
        const uint16_t chunk = (uint16_t)((length - done < contiguous) ? (length - done) : contiguous);
        const uint8_t status = readPhysical((uint16_t)(_start + logical), destination + done, chunk);
        if (status != RING_OK) return status;
        done = (uint16_t)(done + chunk);
        logical = 0;
    }
    return RING_OK;
}

uint8_t RingEEPROM::writeLogical(uint16_t offset, const uint8_t *source, uint16_t length) {
    if (_ringSize == 0 || source == 0) return ERR_INVALID_ARGUMENT;
    uint32_t logical = offset % _ringSize;
    uint16_t done = 0;
    while (done < length) {
        const uint16_t contiguous = (uint16_t)(_ringSize - logical);
        const uint16_t chunk = (uint16_t)((length - done < contiguous) ? (length - done) : contiguous);
        const uint8_t status = writePhysical((uint16_t)(_start + logical), source + done, chunk);
        if (status != RING_OK) return status;
        done = (uint16_t)(done + chunk);
        logical = 0;
    }
    return RING_OK;
}

uint8_t RingEEPROM::readPhysical(uint16_t address, uint8_t *destination, uint16_t length) {
    if ((uint32_t)address + length > _capacity) return ERR_ADDRESS_OUT_OF_RANGE;
    if (length == 0) return RING_OK;

    if (_device == RING_EEPROM_INTERNAL) {
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
        for (uint16_t i = 0; i < length; ++i) destination[i] = EEPROM.read(address + i);
        return RING_OK;
#else
        return ERR_DEVICE_NOT_SUPPORTED;
#endif
    }

    uint16_t done = 0;
    while (done < length) {
        const uint16_t chunk = (uint16_t)(((uint32_t)(length - done) > 30UL) ? 30U : (length - done));
#if RING_EEPROM_USE_HARDWARE_I2C
        Wire.beginTransmission((uint8_t)RING_EEPROM_I2C_ADDRESS);
        Wire.write((uint8_t)((address + done) >> 8));
        Wire.write((uint8_t)(address + done));
        const uint8_t txStatus = Wire.endTransmission(false);
        if (txStatus != 0) return mapWireError(txStatus);
        Wire.clearWireTimeoutFlag();
        const uint8_t received = (uint8_t)Wire.requestFrom((uint8_t)RING_EEPROM_I2C_ADDRESS, (uint8_t)chunk);
        if (Wire.getWireTimeoutFlag()) {
            while (Wire.available()) (void)Wire.read();
            return ERR_I2C_TIMEOUT;
        }
        if (received != chunk) {
            while (Wire.available()) (void)Wire.read();
            return ERR_I2C_NO_RESPONSE;
        }
        for (uint16_t i = 0; i < chunk; ++i) {
            if (!Wire.available()) return ERR_I2C_NO_RESPONSE;
            destination[done + i] = (uint8_t)Wire.read();
        }
#else
        const uint8_t softStatus = softReadMemory((uint16_t)(address + done), destination + done, chunk);
        if (softStatus != RING_OK) return softStatus;
#endif
        done = (uint16_t)(done + chunk);
    }
    return RING_OK;
}

uint8_t RingEEPROM::writePhysical(uint16_t address, const uint8_t *source, uint16_t length) {
    if ((uint32_t)address + length > _capacity) return ERR_ADDRESS_OUT_OF_RANGE;
    if (length == 0) return RING_OK;

    if (_device == RING_EEPROM_INTERNAL) {
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
        for (uint16_t i = 0; i < length; ++i) EEPROM.update(address + i, source[i]);
        return RING_OK;
#else
        return ERR_DEVICE_NOT_SUPPORTED;
#endif
    }

    uint16_t done = 0;
    while (done < length) {
        const uint16_t current = (uint16_t)(address + done);
        const uint16_t pageRemaining = (uint16_t)(32U - (current % 32U));
        uint16_t chunk = (uint16_t)((length - done < pageRemaining) ? (length - done) : pageRemaining);
        if (chunk > 30U) chunk = 30U;
#if RING_EEPROM_USE_HARDWARE_I2C
        Wire.beginTransmission((uint8_t)RING_EEPROM_I2C_ADDRESS);
        Wire.write((uint8_t)(current >> 8));
        Wire.write((uint8_t)current);
        for (uint16_t i = 0; i < chunk; ++i) Wire.write(source[done + i]);
        const uint8_t txStatus = Wire.endTransmission();
        if (txStatus != 0) return mapWireError(txStatus);

        const uint32_t started = millis();
        bool ready = false;
        while ((uint32_t)(millis() - started) < RING_EEPROM_I2C_TIMEOUT_MS) {
            Wire.beginTransmission((uint8_t)RING_EEPROM_I2C_ADDRESS);
            if (Wire.endTransmission() == 0) {
                ready = true;
                break;
            }
            delay(1);
        }
        if (!ready) return ERR_I2C_TIMEOUT;
#else
        const uint8_t softStatus = softWriteMemory(current, source + done, chunk);
        if (softStatus != RING_OK) return softStatus;
#endif
        done = (uint16_t)(done + chunk);
    }
    return RING_OK;
}
