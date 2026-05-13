#!/bin/bash
# Append usbcore.autosuspend=-1 to the BLS boot loader entry options line.
# Idempotent: if already present, does nothing.
# Backup written next to the original with .bak.w2_01 suffix.
set -euo pipefail

BLS_DIR="/boot/loader/entries"
BLS_FILE=$(ls "$BLS_DIR"/ostree-*.conf 2>/dev/null | head -n1 || true)

if [ -z "$BLS_FILE" ] || [ ! -f "$BLS_FILE" ]; then
  echo "ERROR: no BLS entry found in $BLS_DIR" >&2
  exit 2
fi

echo "BLS_FILE=$BLS_FILE"
echo "--- BEFORE ---"
cat "$BLS_FILE"

if grep -q 'usbcore.autosuspend=-1' "$BLS_FILE"; then
  echo "--- already present, no change ---"
  exit 0
fi

cp -a "$BLS_FILE" "${BLS_FILE}.bak.w2_01"
sed -i '/^options /s/$/ usbcore.autosuspend=-1/' "$BLS_FILE"

echo "--- AFTER ---"
cat "$BLS_FILE"
echo "--- backup at ${BLS_FILE}.bak.w2_01 ---"
