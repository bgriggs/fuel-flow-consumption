#!/bin/bash
# Compile and link the sketch for the ATmega32U4, without the Arduino IDE.
#
# Reproduces what the IDE does: concatenate the .ino files into one
# translation unit behind #include <Arduino.h>, then build it against the AVR
# core and the libraries this sketch uses. Verifies the firmware still builds
# and reports the flash and RAM cost.
#
# Usage: bash tools/avr-build.sh
set -u

SKETCH_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARDUINO15="${ARDUINO15:-$LOCALAPPDATA/Arduino15}"
SKETCHBOOK="${SKETCHBOOK:-$USERPROFILE/OneDrive/Documents/Arduino}"

AVR_GCC_DIR="$(ls -d "$ARDUINO15"/packages/arduino/tools/avr-gcc/*/bin 2>/dev/null | head -1)"
CORE_DIR="$(ls -d "$ARDUINO15"/packages/arduino/hardware/avr/*/cores/arduino 2>/dev/null | head -1)"
VARIANT_DIR="$(ls -d "$ARDUINO15"/packages/arduino/hardware/avr/*/variants/leonardo 2>/dev/null | head -1)"
LIB_DIR="$(ls -d "$ARDUINO15"/packages/arduino/hardware/avr/*/libraries 2>/dev/null | head -1)"
MCP_DIR="$(ls -d "$SKETCHBOOK"/libraries/Longan_Labs_Arduino_CAN_Bus_Library_for_MCP2515 2>/dev/null | head -1)"

for v in AVR_GCC_DIR CORE_DIR VARIANT_DIR LIB_DIR MCP_DIR; do
  if [ -z "${!v}" ] || [ ! -d "${!v}" ]; then
    echo "ERROR: could not locate $v" >&2
    echo "  Set ARDUINO15 and SKETCHBOOK if your install is elsewhere." >&2
    exit 1
  fi
done

CXX="$AVR_GCC_DIR/avr-g++"
CC="$AVR_GCC_DIR/avr-gcc"
AR="$AVR_GCC_DIR/avr-gcc-ar"
SIZE="$AVR_GCC_DIR/avr-size"

BUILD="${BUILD_DIR:-$SKETCH_DIR/build-avr}"
rm -rf "$BUILD"
mkdir -p "$BUILD/obj"

# The CANBed Elite is an ATmega32U4 at 16 MHz, so the Leonardo variant applies.
MCU_FLAGS="-mmcu=atmega32u4 -DF_CPU=16000000L -DARDUINO=10819 \
  -DARDUINO_AVR_LEONARDO -DARDUINO_ARCH_AVR -DUSB_VID=0x2341 -DUSB_PID=0x8036 \
  -DUSB_MANUFACTURER=\"Unknown\" -DUSB_PRODUCT=\"CANBed\""

# -I"$SKETCH_DIR" mirrors the IDE putting the sketch folder on the include
# path, which is what makes #include "src/Firmware.h" resolve.
INCLUDES="-I$SKETCH_DIR -I$CORE_DIR -I$VARIANT_DIR -I$LIB_DIR/SPI/src -I$LIB_DIR/EEPROM/src -I$MCP_DIR"

COMMON="-Os -w -ffunction-sections -fdata-sections -MMD -flto $MCU_FLAGS $INCLUDES"
CXXFLAGS="$COMMON -fno-exceptions -fno-threadsafe-statics -fpermissive -std=gnu++11"
CFLAGS="$COMMON -std=gnu11"

fail=0

compile_cxx() {  # <source> <object> [extra warning flags]
  # shellcheck disable=SC2086
  if ! "$CXX" -c $CXXFLAGS ${3:-} "$1" -o "$2" 2>"$BUILD/err.txt"; then
    echo "--- FAILED: $1"
    cat "$BUILD/err.txt"
    fail=1
  elif [ -s "$BUILD/err.txt" ]; then
    echo "--- warnings: $1"
    cat "$BUILD/err.txt"
  fi
}

echo "== Building AVR core =="
core_objs=()
i=0
while IFS= read -r src; do
  obj="$BUILD/obj/core_$i.o"
  case "$src" in
    *.c)   "$CC" -c $CFLAGS "$src" -o "$obj" 2>/dev/null || fail=1 ;;
    *.cpp) "$CXX" -c $CXXFLAGS "$src" -o "$obj" 2>/dev/null || fail=1 ;;
    *.S)   "$CC" -c -x assembler-with-cpp $CFLAGS "$src" -o "$obj" 2>/dev/null || fail=1 ;;
  esac
  core_objs+=("$obj")
  i=$((i + 1))
done < <(find "$CORE_DIR" -maxdepth 1 \( -name '*.c' -o -name '*.cpp' -o -name '*.S' \))

echo "== Building libraries =="
lib_objs=()
i=0
for src in "$LIB_DIR"/SPI/src/*.cpp "$LIB_DIR"/EEPROM/src/*.cpp "$MCP_DIR"/*.cpp; do
  [ -e "$src" ] || continue
  obj="$BUILD/obj/lib_$i.o"
  "$CXX" -c $CXXFLAGS "$src" -o "$obj" 2>/dev/null || { echo "FAILED: $src"; fail=1; }
  lib_objs+=("$obj")
  i=$((i + 1))
done

# The project's own sources are held to real warning flags; the core and the
# third-party library are not ours to clean up.
STRICT="-Wall -Wextra"

echo "== Building src/ =="
src_objs=()
i=0
for src in "$SKETCH_DIR"/src/*.cpp; do
  [ -e "$src" ] || continue
  obj="$BUILD/obj/src_$i.o"
  compile_cxx "$src" "$obj" "$STRICT"
  src_objs+=("$obj")
  i=$((i + 1))
done

echo "== Building sketch =="
# The IDE concatenates every .ino into one file: the sketch named after the
# folder first, then the rest alphabetically. The order used here differs, and
# deliberately so - the code declares everything it shares in src/Firmware.h
# rather than relying on concatenation order, and building in any order must
# produce the same image.
SKETCH_CPP="$BUILD/sketch.cpp"
{
  echo '#include <Arduino.h>'
  for ino in fuel-usage.ino fuel-flow.ino commands.ino; do
    echo "#line 1 \"$ino\""
    cat "$SKETCH_DIR/$ino"
  done
} > "$SKETCH_CPP"
compile_cxx "$SKETCH_CPP" "$BUILD/obj/sketch.o" "$STRICT"

if [ "$fail" -ne 0 ]; then
  echo
  echo "BUILD FAILED"
  exit 1
fi

echo "== Linking =="
"$AR" rcs "$BUILD/core.a" "${core_objs[@]}" 2>/dev/null
if ! "$CXX" -Os -flto -fuse-linker-plugin -Wl,--gc-sections $MCU_FLAGS \
     -o "$BUILD/firmware.elf" \
     "$BUILD/obj/sketch.o" "${src_objs[@]}" "${lib_objs[@]}" \
     "$BUILD/core.a" -L"$BUILD" -lm 2>"$BUILD/link.txt"; then
  echo "LINK FAILED"
  cat "$BUILD/link.txt"
  exit 1
fi
[ -s "$BUILD/link.txt" ] && cat "$BUILD/link.txt"

echo
echo "== Size (ATmega32U4: 28672 bytes flash usable, 2560 bytes RAM) =="
"$SIZE" -A "$BUILD/firmware.elf" | awk '
  /^\.text/   { text = $2 }
  /^\.data/   { data = $2 }
  /^\.bss/    { bss  = $2 }
  END {
    printf "  Flash (text+data): %6d bytes  (%.1f%% of 28672)\n", text+data, (text+data)*100/28672
    printf "  RAM   (data+bss):  %6d bytes  (%.1f%% of 2560)\n", data+bss, (data+bss)*100/2560
    printf "  NOTE: RAM figure excludes stack and heap growth at run time.\n"
  }'
echo
echo "BUILD OK"
