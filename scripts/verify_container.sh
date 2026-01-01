#!/bin/bash
# Quick test to verify container setup

echo "=== ADAS ECU Container Verification ==="
echo ""

echo "✓ Checking CMake..."
cmake --version || echo "❌ CMake not found"

echo ""
echo "✓ Checking GCC..."
g++ --version | head -n 1 || echo "❌ GCC not found"

echo ""
echo "✓ Checking Python..."
python3 --version || echo "❌ Python not found"

echo ""
echo "✓ Checking Eigen..."
ls /usr/include/eigen3 > /dev/null 2>&1 && echo "Eigen: OK" || echo "❌ Eigen not found"

echo ""
echo "✓ Checking FreeRTOS..."
ls /usr/include/freertos > /dev/null 2>&1 && echo "FreeRTOS: OK" || echo "❌ FreeRTOS not found"

echo ""
echo "✓ Checking CAN utils..."
which cansend > /dev/null 2>&1 && echo "CAN utils: OK" || echo "❌ CAN utils not found"

echo ""
echo "✓ Checking python-can..."
python3 -c "import can; print('python-can: OK')" 2>/dev/null || echo "❌ python-can not found"

echo ""
echo "=== Container is ready! ==="
