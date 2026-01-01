#!/bin/bash
# Build script for ADAS ECU project

set -e  # Exit on error

echo "=========================================="
echo "  ADAS ECU Build Script"
echo "=========================================="

# Clean previous build
if [ -d "build" ]; then
    echo "[1/4] Cleaning previous build..."
    rm -rf build
fi

# Create build directory
echo "[2/4] Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "[3/4] Configuring with CMake..."
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..

# Build
echo "[4/4] Building project..."
make -j$(nproc)

echo ""
echo "=========================================="
echo "  Build completed successfully!"
echo "=========================================="
echo "Executable: build/adas_ecu"
echo ""
echo "To run:"
echo "  1. Set up vcan0: sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0"
echo "  2. Run simulator: python3 scripts/simulate_sensors.py"
echo "  3. Run ECU: ./build/adas_ecu"
echo "=========================================="
