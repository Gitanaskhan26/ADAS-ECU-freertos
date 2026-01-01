#!/bin/bash
# Script to send simulated Lidar data over CAN

INTERFACE="vcan0"
LIDAR_ID="100"  # 0x100

# Send lidar position data (x, y as floats)
# Format: 4 bytes x, 4 bytes y

send_lidar_data() {
    x=$1
    y=$2
    
    # Convert float to hex (using printf and xxd)
    # This is a simplified example - you may need python/C helper for accurate float conversion
    
    # For now, send dummy hex values
    cansend $INTERFACE ${LIDAR_ID}#0000803F0000004F
}

echo "Sending Lidar data to $INTERFACE..."

# Simulate vehicle movement
for i in {1..100}; do
    x=$(echo "scale=2; $i * 0.1" | bc)
    y=$(echo "scale=2; $i * 0.05" | bc)
    
    echo "Sending: x=$x, y=$y"
    send_lidar_data $x $y
    
    sleep 0.01  # 10ms (100 Hz)
done

echo "Done."
