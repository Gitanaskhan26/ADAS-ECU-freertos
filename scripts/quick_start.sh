#!/bin/bash
# Quick Start Guide for Enhanced ADAS ECU

echo "=============================================="
echo "  ADAS ECU - Enhanced Features Demo"
echo "=============================================="
echo ""

# Build project
echo "[1/5] Building project..."
./scripts/build.sh

# Run tests
echo ""
echo "[2/5] Running unit tests..."
cd build
echo "  • Kalman Filter Tests:"
./tests/test_kalman
echo ""
echo "  • CAN Interface Tests:"
./tests/test_can_interface
echo ""
echo "  • Safety Function Tests:"
./tests/test_safety
cd ..

# Install Python dependencies
echo ""
echo "[3/5] Installing Python dependencies..."
python3 -m pip install -q matplotlib pandas numpy 2>/dev/null || echo "Note: Install matplotlib, pandas, numpy for visualization"

echo ""
echo "[4/5] System ready!"
echo ""
echo "=============================================="
echo "  Features Enabled:"
echo "=============================================="
echo "✓ Kalman filter sensor fusion"
echo "✓ Collision detection (TTC-based)"
echo "✓ Data logging to CSV files"
echo "✓ Multi-sensor support (Lidar, Radar, Camera, Ultrasonic)"
echo "✓ Safety watchdog monitoring"
echo "✓ Comprehensive test suite"
echo ""
echo "=============================================="
echo "  Quick Commands:"
echo "=============================================="
echo ""
echo "# Run ECU (creates logs/ directory)"
echo "./build/adas_ecu"
echo ""
echo "# Visualize results (after running ECU)"
echo "python3 scripts/visualize_tracking.py"
echo ""
echo "# View generated plots"
echo "ls plots/"
echo ""
echo "# Analyze log data"
echo "cat logs/collision_detection.csv"
echo "cat logs/safety_events.csv"
echo ""
echo "=============================================="
echo ""
echo "[5/5] For SocketCAN (Linux only):"
echo "  modprobe vcan"
echo "  ip link add dev vcan0 type vcan"
echo "  ip link set up vcan0"
echo "  python3 scripts/simulate_sensors.py"
echo ""
echo "Ready! 🚀"
