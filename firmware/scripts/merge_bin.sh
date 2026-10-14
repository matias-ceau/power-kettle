#!/usr/bin/env bash
# Merge bootloader + partitions + boot_app0 + app into one flashable image
# (firmware/.pio/build/c3/merged.bin, flashed at offset 0 by ESP Web Tools).
# Run after `pio run -e c3`, from anywhere.
set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$FW_DIR/.pio/build/c3"
CORE_DIR="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
BOOT_APP0="$CORE_DIR/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
OUT="$BUILD/merged.bin"

for f in "$BUILD/bootloader.bin" "$BUILD/partitions.bin" "$BOOT_APP0" "$BUILD/firmware.bin"; do
  if [ ! -f "$f" ]; then
    echo "merge_bin.sh: missing input: $f" >&2
    echo "merge_bin.sh: run 'pio run -e c3' in $FW_DIR first" >&2
    exit 1
  fi
done

pio pkg exec -p tool-esptoolpy -- esptool.py --chip esp32c3 merge_bin -o "$OUT" \
  0x0 "$BUILD/bootloader.bin" \
  0x8000 "$BUILD/partitions.bin" \
  0xe000 "$BOOT_APP0" \
  0x10000 "$BUILD/firmware.bin"

if [ ! -s "$OUT" ]; then
  echo "merge_bin.sh: $OUT was not produced" >&2
  exit 1
fi
echo "merge_bin.sh: wrote $OUT ($(wc -c < "$OUT") bytes)"
