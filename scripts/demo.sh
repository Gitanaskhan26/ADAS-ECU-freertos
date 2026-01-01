#!/bin/bash
# Demo script to run ADAS ECU system
# This creates sample log data for visualization

echo "=============================================="
echo "  ADAS ECU Demo - Creating Sample Data"
echo "=============================================="
echo ""

# Note: This would normally run with actual CAN data
# For demo purposes, we'll show what commands to use

echo "To run the full system with SocketCAN on Linux:"
echo ""
echo "Terminal 1 - Setup CAN:"
echo "  modprobe vcan"
echo "  ip link add dev vcan0 type vcan"
echo "  ip link set up vcan0"
echo ""
echo "Terminal 2 - Start sensor simulator:"
echo "  python3 scripts/simulate_sensors.py"
echo ""
echo "Terminal 3 - Run ADAS ECU:"
echo "  ./build/adas_ecu"
echo ""
echo "Terminal 4 - Monitor (optional):"
echo "  candump vcan0"
echo ""
echo "=============================================="
echo ""
echo "After running the ECU, visualize results:"
echo "  python3 scripts/visualize_tracking.py"
echo ""
echo "View generated plots:"
echo "  ls -lh plots/"
echo ""
echo "Note: This Docker environment doesn't have CAN support."
echo "Run on native Linux for full SocketCAN functionality."
echo ""
