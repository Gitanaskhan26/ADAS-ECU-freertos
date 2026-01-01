"""
Accurate CAN sensor simulator using Python
Sends Lidar and Radar data with proper float encoding
"""

from typing import Any
import struct
import time
import math

try:
    import can  # type: ignore
except ImportError:
    print("ERROR: python-can not installed. Run: pip install python-can")
    exit(1)

def send_lidar_frame(bus: Any, x: float, y: float) -> None:
    """Send lidar position data (x, y)"""
    data = struct.pack('<ff', x, y)  # Little-endian floats
    msg = can.Message(arbitration_id=0x100, data=data, is_extended_id=False)
    bus.send(msg)

def send_radar_frame(bus: Any, vx: float, vy: float) -> None:
    """Send radar velocity data (vx, vy)"""
    data = struct.pack('<ff', vx, vy)  # Little-endian floats
    msg = can.Message(arbitration_id=0x200, data=data, is_extended_id=False)
    bus.send(msg)

def main() -> None:
    # Connect to virtual CAN
    try:
        bus = can.interface.Bus(channel='vcan0', interface='socketcan')
    except OSError as e:
        print(f"ERROR: Cannot connect to vcan0: {e}")
        print("\nThis requires:")
        print("  1. Linux kernel with CAN support")
        print("  2. Virtual CAN interface setup:")
        print("     modprobe vcan")
        print("     ip link add dev vcan0 type vcan")
        print("     ip link set up vcan0")
        print("\nNote: Docker on macOS doesn't support CAN kernel modules.")
        print("Run this on native Linux instead.")
        return
    
    print("Starting CAN sensor simulator on vcan0...")
    print("Press Ctrl+C to stop\n")
    
    t = 0.0
    dt = 0.01  # 10ms period (100 Hz)
    
    try:
        while True:
            # Simulate circular motion
            radius = 5.0
            angular_vel = 0.5  # rad/s
            
            x = radius * math.cos(angular_vel * t)
            y = radius * math.sin(angular_vel * t)
            vx = -radius * angular_vel * math.sin(angular_vel * t)
            vy = radius * angular_vel * math.cos(angular_vel * t)
            
            # Send frames
            send_lidar_frame(bus, x, y)
            send_radar_frame(bus, vx, vy)
            
            print(f"t={t:.2f}s | Lidar: x={x:6.2f}, y={y:6.2f} | Radar: vx={vx:6.2f}, vy={vy:6.2f}")
            
            t += dt
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped.")
        bus.shutdown()

if __name__ == "__main__":
    main()
