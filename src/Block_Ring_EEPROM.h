#ifndef BLOCK_RING_EEPROM_H
#define BLOCK_RING_EEPROM_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <type_traits>

/* ============================================================================
   RING_EEPROM — ЕДИНАЯ КАРТА КОДОВ РЕЗУЛЬТАТА

   0 всегда означает успех. Каждый ненулевой код имеет одну глобальную
   причину и не зависит от того, какая публичная операция его вернула.
   Числовые значения опубликованных кодов не переиспользовать.
   ============================================================================ */
enum BlockRingEEPROMResult : uint8_t {
    BLOCK_RING_OK                         = 0,
    BLOCK_ERR_UNKNOWN_DEVICE              = 1,
    BLOCK_ERR_DEVICE_NOT_SUPPORTED        = 2,
    BLOCK_ERR_I2C_INIT_FAILED             = 3,
    BLOCK_ERR_I2C_NO_RESPONSE             = 4,
    BLOCK_ERR_I2C_TIMEOUT                 = 5,
    BLOCK_ERR_INVALID_ADDRESS             = 6,
    BLOCK_ERR_ADDRESS_OUT_OF_RANGE        = 7,
    BLOCK_ERR_INVALID_RANGE               = 8,
    BLOCK_ERR_SIGNATURE_INVALID           = 9,
    BLOCK_ERR_SIGNATURE_TOO_LONG          = 10,
    BLOCK_ERR_NOT_INITIALIZED             = 11,
    BLOCK_ERR_NO_SPACE                    = 12,
    BLOCK_ERR_READ_FAILED                 = 13,
    BLOCK_ERR_WRITE_FAILED                = 14,
    BLOCK_ERR_WRITE_VERIFY                = 15,
    BLOCK_ERR_READ_CRC_FAILED             = 16,
    BLOCK_ERR_READ_NO_RECORD              = 17,
    BLOCK_ERR_INTERNAL_EEPROM_UNAVAILABLE = 18,
    BLOCK_ERR_INVALID_ARGUMENT            = 19
};

/* Тип памяти, который пользователь выбирает в init(). */
enum BlockRingEEPROMDevice : uint8_t {
    BLOCK_INTERNAL = 0,
    BLOCK_AT24C32 = 0x32,
    BLOCK_AT24C64 = 0x64
};

/* ============================================================================
   НАСТРОЙКИ ПО УМОЛЧАНИЮ — МОЖНО ПЕРЕОПРЕДЕЛИТЬ ДО #include <Block_Ring_EEPROM.h>
   ============================================================================ */
#ifndef BLOCK_RING_EEPROM_AT24C32_SIZE
#define BLOCK_RING_EEPROM_AT24C32_SIZE 4096
#endif

#ifndef BLOCK_RING_EEPROM_AT24C64_SIZE
#define BLOCK_RING_EEPROM_AT24C64_SIZE 8192
#endif

#ifndef BLOCK_RING_EEPROM_I2C_ADDRESS
#define BLOCK_RING_EEPROM_I2C_ADDRESS 0x50
#endif

#ifndef BLOCK_RING_EEPROM_SDA
#define BLOCK_RING_EEPROM_SDA A4
#endif

#ifndef BLOCK_RING_EEPROM_SCL
#define BLOCK_RING_EEPROM_SCL A5
#endif

#ifndef BLOCK_RING_EEPROM_I2C_CLOCK
#define BLOCK_RING_EEPROM_I2C_CLOCK 100000UL
#endif

#ifndef BLOCK_RING_EEPROM_I2C_TIMEOUT_MS
#define BLOCK_RING_EEPROM_I2C_TIMEOUT_MS 25
#endif

#ifndef BLOCK_RING_EEPROM_SOFT_I2C_DELAY_US
#define BLOCK_RING_EEPROM_SOFT_I2C_DELAY_US 5
#endif

#ifndef BLOCK_RING_EEPROM_SOFT_I2C_TIMEOUT_US
#define BLOCK_RING_EEPROM_SOFT_I2C_TIMEOUT_US 1000
#endif

#ifndef BLOCK_RING_EEPROM_FALLBACK_DEPTH
#define BLOCK_RING_EEPROM_FALLBACK_DEPTH 3
#endif

#ifndef BLOCK_RING_EEPROM_CRC8_MAX_BYTES
#define BLOCK_RING_EEPROM_CRC8_MAX_BYTES 30
#endif

#ifndef BLOCK_RING_EEPROM_CRC8_POLY
#define BLOCK_RING_EEPROM_CRC8_POLY 0xE7
#endif

#ifndef BLOCK_RING_EEPROM_CRC16_POLY
#define BLOCK_RING_EEPROM_CRC16_POLY 0x8D95
#endif

#ifndef BLOCK_RING_EEPROM_IO_BUFFER_SIZE
#define BLOCK_RING_EEPROM_IO_BUFFER_SIZE 32
#endif

#ifndef BLOCK_RING_EEPROM_INLINE_SIGNATURE_MAX
#define BLOCK_RING_EEPROM_INLINE_SIGNATURE_MAX 16
#endif

/* ============================================================================
   ПУБЛИЧНЫЙ ИНТЕРФЕЙС
   ============================================================================ */
class BlockRingEEPROM {
public:
    BlockRingEEPROM();

    /* Числовая/скалярная сигнатура копируется внутрь объекта библиотеки. */
    template <typename T>
    uint8_t init(uint8_t device, const T &signature) {
        static_assert(std::is_integral<T>::value || std::is_enum<T>::value,
                      "Block_Ring_EEPROM: scalar signature must be an integer or enum; use a fixed byte/char array for arbitrary signatures");
        return initScalar(device, reinterpret_cast<const uint8_t *>(&signature), sizeof(T), false, 0, 0);
    }

    template <typename T>
    uint8_t init(uint8_t device, const T &signature,
                 uint16_t startAddress, uint16_t endAddress) {
        static_assert(std::is_integral<T>::value || std::is_enum<T>::value,
                      "Block_Ring_EEPROM: scalar signature must be an integer or enum; use a fixed byte/char array for arbitrary signatures");
        return initScalar(device, reinterpret_cast<const uint8_t *>(&signature), sizeof(T), true,
                          startAddress, endAddress);
    }

    /* Строковый литерал/char-массив: завершающий нулевой байт не входит в подпись. */
    template <size_t N>
    uint8_t init(uint8_t device, const char (&signature)[N]) {
        static_assert(N > 1, "Block_Ring_EEPROM: signature string must not be empty");
        return initArray(device, reinterpret_cast<const uint8_t *>(signature), (uint16_t)(N - 1), false, 0, 0);
    }

    template <size_t N>
    uint8_t init(uint8_t device, const char (&signature)[N],
                 uint16_t startAddress, uint16_t endAddress) {
        static_assert(N > 1, "Block_Ring_EEPROM: signature string must not be empty");
        return initArray(device, reinterpret_cast<const uint8_t *>(signature), (uint16_t)(N - 1), true,
                         startAddress, endAddress);
    }

    /* Любой другой массив фиксированного размера сохраняется целиком. */
    template <typename T, size_t N>
    uint8_t init(uint8_t device, const T (&signature)[N]) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Block_Ring_EEPROM: signature array element type must be trivially copyable");
        static_assert(sizeof(T) * N <= 65535UL, "Block_Ring_EEPROM: signature is too long");
        return initArray(device, reinterpret_cast<const uint8_t *>(signature),
                         (uint16_t)(sizeof(T) * N), false, 0, 0);
    }

    template <typename T, size_t N>
    uint8_t init(uint8_t device, const T (&signature)[N],
                 uint16_t startAddress, uint16_t endAddress) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Block_Ring_EEPROM: signature array element type must be trivially copyable");
        static_assert(sizeof(T) * N <= 65535UL, "Block_Ring_EEPROM: signature is too long");
        return initArray(device, reinterpret_cast<const uint8_t *>(signature),
                         (uint16_t)(sizeof(T) * N), true, startAddress, endAddress);
    }

    /* Вариант для динамически заданной последовательности байтов.
       Память signature должна оставаться доступной до повторного init(). */
    uint8_t init(uint8_t device, const uint8_t *signature, uint16_t signatureLength);
    uint8_t init(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                 uint16_t startAddress, uint16_t endAddress);

    template <typename T>
    uint8_t read(T &data) {
        static_assert(std::is_trivially_copyable<T>::value && !std::is_pointer<T>::value,
                      "Block_Ring_EEPROM: read() supports only fixed-size trivially-copyable objects, not pointers");
        static_assert(sizeof(T) <= 65535UL, "Block_Ring_EEPROM: data object is too large");
        return readRaw(reinterpret_cast<uint8_t *>(&data), (uint16_t)sizeof(T));
    }

    template <typename T>
    uint8_t write(const T &data) {
        static_assert(std::is_trivially_copyable<T>::value && !std::is_pointer<T>::value,
                      "Block_Ring_EEPROM: write() supports only fixed-size trivially-copyable objects, not pointers");
        static_assert(sizeof(T) <= 65535UL, "Block_Ring_EEPROM: data object is too large");
        return writeRaw(reinterpret_cast<const uint8_t *>(&data), (uint16_t)sizeof(T));
    }

private:
    struct Candidate {
        uint16_t offset;
        uint16_t sequence;
    };

    uint8_t initScalar(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                       bool hasRange, uint16_t startAddress, uint16_t endAddress);
    uint8_t initArray(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                      bool hasRange, uint16_t startAddress, uint16_t endAddress);
    uint8_t initCommon(uint8_t device, const uint8_t *signature, uint16_t signatureLength,
                       bool copySignature, bool hasRange, uint16_t startAddress, uint16_t endAddress);

    uint8_t readRaw(uint8_t *destination, uint16_t dataLength);
    uint8_t writeRaw(const uint8_t *source, uint16_t dataLength);

    uint8_t selectLayout(uint16_t dataLength);
    uint8_t findCandidates(Candidate *candidates, uint8_t &count, bool &signatureSeen);
    uint8_t verifyRecord(uint16_t startOffset, uint16_t expectedSequence,
                         uint8_t *destinationOrNull);
    uint8_t scanSignature(uint16_t offset, bool &matches);
    uint8_t readSequence(uint16_t offset, uint16_t &sequence);
    uint8_t writeRecord(uint16_t startOffset, uint16_t sequence, const uint8_t *data);

    uint8_t readLogical(uint16_t offset, uint8_t *destination, uint16_t length);
    uint8_t writeLogical(uint16_t offset, const uint8_t *source, uint16_t length);
    uint8_t readPhysical(uint16_t address, uint8_t *destination, uint16_t length);
    uint8_t writePhysical(uint16_t address, const uint8_t *source, uint16_t length);

    uint8_t crcUpdate(uint8_t crc, uint8_t value) const;
    uint16_t crcUpdate(uint16_t crc, uint8_t value) const;
    uint32_t gcd(uint32_t a, uint32_t b) const;
    bool isNewer(uint16_t a, uint16_t b) const;
    void insertCandidate(Candidate *candidates, uint8_t &count, uint16_t offset, uint16_t sequence);

    uint8_t _device;
    uint8_t _initialized;
    uint8_t _softI2C;
    uint16_t _capacity;
    uint16_t _start;
    uint16_t _end;
    uint16_t _ringSize;

    const uint8_t *_signature;
    uint16_t _signatureLength;
    uint8_t _signatureInline[BLOCK_RING_EEPROM_INLINE_SIGNATURE_MAX];

    uint8_t _sequenceBytes;
    uint8_t _crcBytes;
    uint16_t _recordLength;
    uint16_t _step;
    uint16_t _dataLength;

    uint8_t _ioBuffer[BLOCK_RING_EEPROM_IO_BUFFER_SIZE];

    // Кэш последней успешно проверенной записи.
    bool _hasLastGood;
    uint16_t _lastGoodOffset;
    uint16_t _lastGoodSequence;
};

extern BlockRingEEPROM Block_Ring_EEPROM;

#endif /* BLOCK_RING_EEPROM_H */
