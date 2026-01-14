#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./build.sh                # build for pico2 (default)
#   PICO_BOARD=pico ./build.sh  # build for pico (RP2040)

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
BOARD="${PICO_BOARD:-pico2}"

# Prefer Ninja if installed, else use Unix Makefiles
GENERATOR=""
if command -v ninja >/dev/null 2>&1; then
  GENERATOR="-G Ninja"
fi

mkdir -p "$BUILD_DIR"

# Configure if needed (or if CMakeCache missing)
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
  echo "[build] Configuring (board=${BOARD})..."
  cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" \
    ${GENERATOR} \
    -DPICO_BOARD="${BOARD}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
else
  echo "[build] Using existing configuration (board cached in build/)."
fi

echo "[build] Building..."
cmake --build "$BUILD_DIR" -- -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu)"

# Make clangd happy (optional but nice)
if [[ -f "$BUILD_DIR/compile_commands.json" ]]; then
  ln -sf "$BUILD_DIR/compile_commands.json" "$PROJECT_ROOT/compile_commands.json"
fi

echo "[build] Done."
