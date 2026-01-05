#!/usr/bin/env python3
"""
Mock sensor data generator for testing without SocketCAN.
Creates CSV files that simulate sensor data for visualization testing.
"""

import time
import math
import csv
from pathlib import Path

def generate_mock_data(duration_seconds=10, output_dir='logs'):
    """Generate mock sensor and state data."""
    
    Path(output_dir).mkdir(exist_ok=True)
    
    print(f"Generating {duration_seconds}s of mock sensor data...")
    print(f"Output directory: {output_dir}/")
    
    # Open CSV files
    sensor_file = open(f'{output_dir}/sensor_data.csv', 'w', newline='')
    state_file = open(f'{output_dir}/state_estimate.csv', 'w', newline='')
    collision_file = open(f'{output_dir}/collision_detection.csv', 'w', newline='')
    safety_file = open(f'{output_dir}/safety_events.csv', 'w', newline='')
    
    sensor_writer = csv.writer(sensor_file)
    state_writer = csv.writer(state_file)
    collision_writer = csv.writer(collision_file)
    safety_writer = csv.writer(safety_file)
    
    # Write headers
    sensor_writer.writerow(['timestamp_us', 'x', 'y', 'vx', 'vy'])
    state_writer.writerow(['timestamp_us', 'x', 'y', 'vx', 'vy'])
    collision_writer.writerow(['timestamp_us', 'risk_level', 'ttc', 'distance', 'is_approaching'])
    safety_writer.writerow(['timestamp_us', 'event'])
    
    dt = 0.02  # 20ms (50 Hz)
    t = 0.0
    
    # Simulate circular motion approaching origin
    radius = 8.0
    angular_vel = 0.3  # rad/s
    
    warning_logged = False
    critical_logged = False
    
    while t < duration_seconds:
        timestamp_us = int(t * 1e6)
        
        # Position (circular path, moving closer to origin)
        shrink_factor = max(0.2, 1.0 - t / duration_seconds)
        x = radius * shrink_factor * math.cos(angular_vel * t)
        y = radius * shrink_factor * math.sin(angular_vel * t)
        
        # Velocity
        vx = -radius * shrink_factor * angular_vel * math.sin(angular_vel * t)
        vy = radius * shrink_factor * angular_vel * math.cos(angular_vel * t)
        
        # Add some noise to sensor data
        sensor_x = x + 0.1 * math.sin(t * 10)
        sensor_y = y + 0.1 * math.cos(t * 8)
        sensor_vx = vx + 0.05 * math.sin(t * 5)
        sensor_vy = vy + 0.05 * math.cos(t * 7)
        
        # Calculate distance and TTC
        distance = math.sqrt(x*x + y*y)
        radial_velocity = (x * vx + y * vy) / distance if distance > 0.01 else 0
        closing_speed = -radial_velocity
        
        is_approaching = closing_speed > 0.1
        ttc = distance / closing_speed if closing_speed > 0.1 else 999.0
        
        # Determine risk level
        if distance < 2.0 or ttc < 1.5:
            risk_level = 'CRITICAL'
            if not critical_logged:
                safety_writer.writerow([timestamp_us, 'COLLISION_CRITICAL'])
                safety_writer.writerow([timestamp_us, 'EMERGENCY_BRAKE'])
                critical_logged = True
        elif ttc < 3.0:
            risk_level = 'WARNING'
            if not warning_logged:
                safety_writer.writerow([timestamp_us, 'COLLISION_WARNING'])
                warning_logged = True
        elif is_approaching:
            risk_level = 'LOW'
        else:
            risk_level = 'NONE'
        
        # Write data
        sensor_writer.writerow([timestamp_us, sensor_x, sensor_y, sensor_vx, sensor_vy])
        state_writer.writerow([timestamp_us, x, y, vx, vy])
        collision_writer.writerow([timestamp_us, risk_level, ttc, distance, 1 if is_approaching else 0])
        
        t += dt
    
    # Close files
    sensor_file.close()
    state_file.close()
    collision_file.close()
    safety_file.close()
    
    print(f"\n✓ Generated {int(t/dt)} data points")
    print(f"✓ Files created in {output_dir}/:")
    print(f"  - sensor_data.csv")
    print(f"  - state_estimate.csv")
    print(f"  - collision_detection.csv")
    print(f"  - safety_events.csv")
    print(f"\nNow run: python3 scripts/visualize_tracking.py")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate mock sensor data for testing')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration in seconds (default: 10)')
    parser.add_argument('--output-dir', default='logs', help='Output directory (default: logs)')
    
    args = parser.parse_args()
    
    generate_mock_data(args.duration, args.output_dir)
