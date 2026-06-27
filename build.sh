#!/bin/bash
# Build script for Osmo Action GPS Remote
# Uses PlatformIO's ESP-IDF installation

set -e

IDF_PATH="/Users/royyanwicaksono/.platformio/packages/framework-espidf"
IDF_PYTHON="/Users/royyanwicaksono/.platformio/penv/.espidf-6.0.1/bin/python3"
TOOLCHAIN_BIN="/Users/royyanwicaksono/.platformio/packages/toolchain-riscv32-esp/bin"
CMAKE_BIN="/Users/royyanwicaksono/.platformio/packages/tool-cmake/bin"
NINJA_BIN="/Users/royyanwicaksono/.platformio/packages/tool-ninja"

export IDF_PATH
export IDF_TOOLS_PATH="/Users/royyanwicaksono/.platformio"
export PATH="${TOOLCHAIN_BIN}:${CMAKE_BIN}:${NINJA_BIN}:${PATH}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ACTION="${1:-build}"

echo "=== Osmo GPS Remote — ESP32-C6 Super Mini ==="
echo "IDF_PATH: $IDF_PATH"
echo "Action: $ACTION"
echo ""

$IDF_PYTHON "$IDF_PATH/tools/idf.py" \
    -C "$SCRIPT_DIR" \
    -DIDF_TARGET=esp32c6 \
    "$@"
