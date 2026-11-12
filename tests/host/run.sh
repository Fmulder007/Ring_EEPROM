#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
BUILD="$ROOT/tests/host/build"
mkdir -p "$BUILD"
CXX=\${CXX:-g++}

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -I"$ROOT/tests/host/include" -I"$ROOT/src" \
  "$ROOT/src/Ring_EEPROM.cpp" "$ROOT/tests/host/host_test.cpp" \
  -o "$BUILD/host_test"
"$BUILD/host_test"

# Also emulate AVR cores where Arduino INTERNAL analog reference equals 2.
"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 -DINTERNAL=2 \
  -I"$ROOT/tests/host/include" -I"$ROOT/src" \
  "$ROOT/src/Ring_EEPROM.cpp" "$ROOT/tests/host/host_test.cpp" \
  -o "$BUILD/host_test_internal2"
"$BUILD/host_test_internal2"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -DRING_EEPROM_SDA=2 -DRING_EEPROM_SCL=3 \
  -I"$ROOT/tests/host/include" -I"$ROOT/src" \
  -c "$ROOT/src/Ring_EEPROM.cpp" -o "$BUILD/soft_i2c.o"

"$CXX" -std=c++11 -Wall -Wextra -Werror \
  -DARDUINO_ARCH_AVR -DE2END=1023 \
  -I"$ROOT/tests/host/include" -I"$ROOT/src" \
  -c "$ROOT/tests/host/api_compile.cpp" -o "$BUILD/api_compile.o"

printf '%s\n' "Ring_EEPROM host tests: OK"
