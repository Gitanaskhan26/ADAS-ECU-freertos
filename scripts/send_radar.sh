#!/bin/bash
# Script to send simulated Radar data over CAN

INTERFACE="vcan0"
RADAR_ID="200"  # 0x200

# Send radar velocity data (vx, vy as floats)
# Format: 4 bytes vx, 4 bytes vy

send_radar_data() {
    vx=$1
    vy=$2
    
    # For now, send dummy hex values
    # In production, use proper float encoding
    cansend $INTERFACE ${RADAR_ID}#0000803F0000003F
}

echo "Sending Radar data to $INTERFACE..."

# Simulate constant velocity
for i in {1..100}; do
    vx="1.0"
    vy="0.5"
    
    echo "Sending: vx=$vx, vy=$vy"
    send_radar_data $vx $vy
    
    sleep 0.01  # 10ms (100 Hz)
done

echo "Done."
