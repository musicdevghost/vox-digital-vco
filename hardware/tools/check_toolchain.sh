#!/usr/bin/env bash
set -euo pipefail

echo "== Detecting OS =="
OS="$(uname -s)"
case "$OS" in
  Darwin)  PLATFORM="macOS";;
  Linux)   PLATFORM="Linux";;
  *)       PLATFORM="Unknown";;
esac
echo "Platform: $PLATFORM"

need() { command -v "$1" >/dev/null 2>&1 || echo "$1"; }

echo "== Checking Arm GNU toolchain =="
MISS=()
for t in arm-none-eabi-gcc arm-none-eabi-g++ arm-none-eabi-objcopy arm-none-eabi-size; do
  if [ -n "$(need "$t")" ]; then MISS+=("$t"); else "$t" --version | head -n1; fi
done

echo "== Checking newlib (link sanity) =="
echo 'int main(){}' | arm-none-eabi-gcc -x c - -specs=nosys.specs -lc -Wl,--gc-sections -nostartfiles -o /tmp/_chk 2>/dev/null \
  && echo "newlib: OK" || MISS+=("newlib (libc/newlib for ARM)")

echo "== Checking flashing tools =="
DFU_OK=0; OPENOCD_OK=0
command -v dfu-util >/dev/null 2>&1 && DFU_OK=1
command -v openocd   >/dev/null 2>&1 && OPENOCD_OK=1
if [ $DFU_OK -eq 0 ] && [ $OPENOCD_OK -eq 0 ]; then MISS+=("dfu-util or openocd"); fi

if [ ${#MISS[@]} -gt 0 ]; then
  echo "== Missing: ${MISS[*]}"
  echo "== Install hints =="
  if [ "$PLATFORM" = "macOS" ]; then
    cat <<'MAC'
brew update
brew install --cask gcc-arm-embedded
brew install dfu-util
# optional:
brew install open-ocd
MAC
  elif [ "$PLATFORM" = "Linux" ]; then
    cat <<'LIN'
sudo apt update
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  dfu-util openocd
LIN
  else
    echo "Please install arm-none-eabi toolchain, newlib, and dfu-util or openocd for your OS."
  fi
else
  echo "== Toolchain looks good."
fi

echo "== libDaisy / DaisySP =="
: "${DAISY_DIR:?Set DAISY_DIR to your libDaisy root}"
: "${DAISYSP_DIR:?Set DAISYSP_DIR to your DaisySP root}"
echo "DAISY_DIR=$DAISY_DIR"
echo "DAISYSP_DIR=$DAISYSP_DIR"
test -f "$DAISY_DIR/core/STM32H750IB_flash.lds" && echo "libDaisy: OK" || echo "libDaisy: missing/invalid"
test -d "$DAISYSP_DIR/Source" && echo "DaisySP: OK" || echo "DaisySP: missing/invalid"
