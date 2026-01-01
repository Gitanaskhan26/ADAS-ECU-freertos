#!/bin/bash
# Run ADAS ECU simulation without requiring vcan
# This script demonstrates the system in simulation mode

set -e

echo "=========================================="
echo "  ADAS ECU Simulation (Demo Mode)"
echo "=========================================="
echo ""
echo "⚠️  Note: Running without SocketCAN"
echo "    The ECU will run but won't receive real CAN data"
echo ""

# Check if vcan0 exists
if ip link show vcan0 >/dev/null 2>&1; then
    echo "✓ vcan0 interface found"
    echo ""
    echo "Starting sensor simulator in background..."
    python3 scripts/simulate_sensors.py &
    SIMULATOR_PID=$!
    echo "Simulator PID: $SIMULATOR_PID"
    sleep 2
    
    echo ""
    echo "Starting ADAS ECU..."
    echo "Press Ctrl+C to stop"
    echo "=========================================="
    ./build/adas_ecu
    
    # Cleanup
    kill $SIMULATOR_PID 2>/dev/null || true
else
    echo "✗ vcan0 not available"
    echo ""
    echo "To run with SocketCAN, you need:"
    echo "  1. Native Linux environment (not Docker on macOS)"
    echo "  2. Run: modprobe vcan"
    echo "  3. Run: ip link add dev vcan0 type vcan"
    echo "  4. Run: ip link set up vcan0"
    echo ""
    echo "Alternative: Running ECU in demo mode (no sensor data)..."
    echo "Press Ctrl+C to stop"
    echo "=========================================="
    timeout 10 ./build/adas_ecu || true
fi
