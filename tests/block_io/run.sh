#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
BUILD="$ROOT/tests/block_io/build"
mkdir -p "$BUILD"
CXX=${CXX:-g++}

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -I"$ROOT/tests/block_io/include" -I"$ROOT/src" \
  "$ROOT/src/Block_Ring_EEPROM.cpp" "$ROOT/tests/block_io/host_test.cpp" \
  -o "$BUILD/host_test"
"$BUILD/host_test"

# Also emulate AVR cores where Arduino INTERNAL analog reference equals 2.
"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 -DINTERNAL=2 \
  -I"$ROOT/tests/block_io/include" -I"$ROOT/src" \
  "$ROOT/src/Block_Ring_EEPROM.cpp" "$ROOT/tests/block_io/host_test.cpp" \
  -o "$BUILD/host_test_internal2"
"$BUILD/host_test_internal2"

# Exercise the documented alternative CRC polynomials.
"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -DBLOCK_RING_EEPROM_CRC8_POLY=0xA6 -DBLOCK_RING_EEPROM_CRC16_POLY=0xFDBF \
  -I"$ROOT/tests/block_io/include" -I"$ROOT/src" \
  "$ROOT/src/Block_Ring_EEPROM.cpp" "$ROOT/tests/block_io/host_test.cpp" \
  -o "$BUILD/host_test_alt_crc"
"$BUILD/host_test_alt_crc"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -DBLOCK_RING_EEPROM_SDA=2 -DBLOCK_RING_EEPROM_SCL=3 \
  -I"$ROOT/tests/block_io/include" -I"$ROOT/src" \
  -c "$ROOT/src/Block_Ring_EEPROM.cpp" -o "$BUILD/soft_i2c.o"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -I"$ROOT/tests/block_io/include" -I"$ROOT/src" \
  -c "$ROOT/tests/block_io/api_compile.cpp" -o "$BUILD/api_compile.o"

printf '%s\n' "Block_Ring_EEPROM host tests: OK"
