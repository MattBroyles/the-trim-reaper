#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./upload.sh
#   UF2=build/other.uf2 ./upload.sh

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UF2="${UF2:-${PROJECT_ROOT}/build/ibus_pwm_pico2.uf2}"

if [[ ! -f "$UF2" ]]; then
  echo "[upload] UF2 not found: $UF2"
  echo "        Build first: ./build.sh"
  exit 1
fi

echo "[upload] UF2: $UF2"

# 1) Try picotool "auto" flash (no BOOTSEL button)
if command -v picotool >/dev/null 2>&1; then
  echo "[upload] Trying picotool auto-flash..."
  # Many picotool builds support forcing boot into BOOTSEL during load.
  # Try the most robust sequence.
  set +e
  picotool load -f "$UF2" >/dev/null 2>&1
  RC=$?
  set -e

  if [[ $RC -eq 0 ]]; then
    echo "[upload] picotool load succeeded. Rebooting..."
    picotool reboot >/dev/null 2>&1 || true
    echo "[upload] Done."
    exit 0
  else
    echo "[upload] picotool load -f failed (maybe firmware not cooperative yet)."
  fi
else
  echo "[upload] picotool not found; will try BOOTSEL drive copy."
fi

# 2) BOOTSEL copy fallback (mass storage)
# We search typical mount locations on macOS and Linux for RP2350/RPI-RP2 volumes.

echo "[upload] Looking for BOOTSEL mass-storage volume (plug in while holding BOOTSEL)..."

find_bootsel_mount() {
  local candidates=()

  # macOS
  if [[ "$(uname -s)" == "Darwin" ]]; then
    candidates+=(/Volumes/RP2350 /Volumes/RPI-RP2)
  else
    # Linux: common auto-mount roots
    candidates+=(/run/media/"$USER"/RP2350 /run/media/"$USER"/RPI-RP2)
    candidates+=(/media/"$USER"/RP2350 /media/"$USER"/RPI-RP2)
    candidates+=(/mnt/RP2350 /mnt/RPI-RP2)
  fi

  for p in "${candidates[@]}"; do
    if [[ -d "$p" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

MOUNT="$(find_bootsel_mount || true)"
if [[ -z "${MOUNT}" ]]; then
  echo "[upload] BOOTSEL volume not found."
  echo "        Steps:"
  echo "          1) Unplug Pico"
  echo "          2) Hold BOOTSEL"
  echo "          3) Plug USB"
  echo "          4) Release BOOTSEL"
  echo "        Then re-run: ./upload.sh"
  exit 2
fi

echo "[upload] Found BOOTSEL volume at: $MOUNT"
echo "[upload] Copying UF2..."
cp "$UF2" "$MOUNT"/
sync || true
echo "[upload] Done. The drive should disappear as the Pico reboots."
