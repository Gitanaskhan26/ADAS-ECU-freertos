#!/usr/bin/env python3
"""
Visualization tool for ADAS ECU Kalman filter tracking data.
Plots sensor measurements, state estimates, and collision detection results.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys

def load_log_data(log_dir='logs'):
    """Load all log files into pandas DataFrames."""
    log_path = Path(log_dir)
    
    data = {}
    
    # Load sensor data
    sensor_file = log_path / 'sensor_data.csv'
    if sensor_file.exists():
        data['sensor'] = pd.read_csv(sensor_file)
        data['sensor']['time_s'] = data['sensor']['timestamp_us'] / 1e6
    
    # Load state estimates
    state_file = log_path / 'state_estimate.csv'
    if state_file.exists():
        data['state'] = pd.read_csv(state_file)
        data['state']['time_s'] = data['state']['timestamp_us'] / 1e6
    
    # Load collision detection
    collision_file = log_path / 'collision_detection.csv'
    if collision_file.exists():
        data['collision'] = pd.read_csv(collision_file)
        data['collision']['time_s'] = data['collision']['timestamp_us'] / 1e6
    
    # Load safety events
    safety_file = log_path / 'safety_events.csv'
    if safety_file.exists():
        data['safety'] = pd.read_csv(safety_file)
        data['safety']['time_s'] = data['safety']['timestamp_us'] / 1e6
    
    return data

def plot_trajectory(data, save_path=None):
    """Plot 2D trajectory: sensor measurements vs Kalman filter estimates."""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    if 'sensor' in data:
        ax.scatter(data['sensor']['x'], data['sensor']['y'], 
                  alpha=0.3, s=20, label='Sensor Measurements', c='lightblue')
    
    if 'state' in data:
        ax.plot(data['state']['x'], data['state']['y'], 
               'b-', linewidth=2, label='Kalman Filter Estimate')
        
        # Mark start and end
        ax.plot(data['state']['x'].iloc[0], data['state']['y'].iloc[0], 
               'go', markersize=12, label='Start')
        ax.plot(data['state']['x'].iloc[-1], data['state']['y'].iloc[-1], 
               'ro', markersize=12, label='End')
    
    # Mark obstacle at origin
    ax.plot(0, 0, 'rx', markersize=20, markeredgewidth=3, label='Obstacle')
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Vehicle Trajectory: Kalman Filter Tracking', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.axis('equal')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    return fig

def plot_velocity(data, save_path=None):
    """Plot velocity components over time."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    if 'sensor' in data:
        ax1.scatter(data['sensor']['time_s'], data['sensor']['vx'], 
                   alpha=0.3, s=10, label='Sensor Vx', c='lightcoral')
        ax2.scatter(data['sensor']['time_s'], data['sensor']['vy'], 
                   alpha=0.3, s=10, label='Sensor Vy', c='lightblue')
    
    if 'state' in data:
        ax1.plot(data['state']['time_s'], data['state']['vx'], 
                'r-', linewidth=2, label='Estimated Vx')
        ax2.plot(data['state']['time_s'], data['state']['vy'], 
                'b-', linewidth=2, label='Estimated Vy')
    
    ax1.set_ylabel('Vx (m/s)', fontsize=11)
    ax1.set_title('Velocity Estimation', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    ax2.set_ylabel('Vy (m/s)', fontsize=11)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    return fig

def plot_collision_risk(data, save_path=None):
    """Plot collision detection metrics."""
    if 'collision' not in data:
        print("No collision detection data available")
        return None
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    df = data['collision']
    
    # Risk level (color-coded)
    risk_colors = {'NONE': 'green', 'LOW': 'yellow', 'WARNING': 'orange', 'CRITICAL': 'red'}
    for risk_level, color in risk_colors.items():
        mask = df['risk_level'] == risk_level
        if mask.any():
            ax1.scatter(df[mask]['time_s'], [risk_level] * mask.sum(), 
                       c=color, s=50, label=risk_level, alpha=0.7)
    
    ax1.set_ylabel('Risk Level', fontsize=11)
    ax1.set_title('Collision Detection Status', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Time to collision
    ttc_finite = df[df['ttc'] < 100]['ttc']  # Filter out infinity
    ax2.plot(df[df['ttc'] < 100]['time_s'], ttc_finite, 'b-', linewidth=2)
    ax2.axhline(y=3.0, color='orange', linestyle='--', label='Warning Threshold (3s)')
    ax2.axhline(y=1.5, color='red', linestyle='--', label='Critical Threshold (1.5s)')
    ax2.set_ylabel('TTC (s)', fontsize=11)
    ax2.set_ylim(0, 10)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Distance to obstacle
    ax3.plot(df['time_s'], df['distance'], 'g-', linewidth=2)
    ax3.axhline(y=2.0, color='red', linestyle='--', label='Min Safe Distance (2m)')
    ax3.set_ylabel('Distance (m)', fontsize=11)
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    return fig

def plot_safety_events(data, save_path=None):
    """Plot safety events timeline."""
    if 'safety' not in data or len(data['safety']) == 0:
        print("No safety events logged")
        return None
    
    fig, ax = plt.subplots(figsize=(12, 4))
    
    df = data['safety']
    event_types = df['event'].unique()
    
    from matplotlib import cm
    try:
        colormap = cm.get_cmap('Set3')  # type: ignore
    except AttributeError:
        # Matplotlib 3.7+
        import matplotlib
        colormap = matplotlib.colormaps['Set3']  # type: ignore
    colors = colormap(np.linspace(0, 1, len(event_types)))
    
    for idx, event_type in enumerate(event_types):
        mask = df['event'] == event_type
        ax.scatter(df[mask]['time_s'], [idx] * mask.sum(), 
                  c=[colors[idx]], s=100, label=event_type, marker='s')  # type: ignore
    
    ax.set_yticks(range(len(event_types)))
    ax.set_yticklabels(event_types)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title('Safety Events Timeline', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='x')
    ax.legend(loc='upper right')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    return fig

def generate_report(log_dir='logs', output_dir='plots'):
    """Generate complete visualization report."""
    print("Loading log data...")
    data = load_log_data(log_dir)
    
    if not data:
        print(f"No log data found in {log_dir}")
        return
    
    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)
    
    print("Generating visualizations...")
    
    # Trajectory plot
    if 'state' in data:
        print("  - Trajectory plot")
        plot_trajectory(data, f'{output_dir}/trajectory.png')
    
    # Velocity plot
    if 'state' in data:
        print("  - Velocity plot")
        plot_velocity(data, f'{output_dir}/velocity.png')
    
    # Collision risk plot
    if 'collision' in data:
        print("  - Collision detection plot")
        plot_collision_risk(data, f'{output_dir}/collision_risk.png')
    
    # Safety events plot
    if 'safety' in data:
        print("  - Safety events plot")
        plot_safety_events(data, f'{output_dir}/safety_events.png')
    
    print(f"\n✓ Visualizations saved to {output_dir}/")
    print("\nGenerated plots:")
    print(f"  - {output_dir}/trajectory.png")
    print(f"  - {output_dir}/velocity.png")
    print(f"  - {output_dir}/collision_risk.png")
    print(f"  - {output_dir}/safety_events.png")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Visualize ADAS ECU tracking data')
    parser.add_argument('--log-dir', default='logs', help='Directory containing log files')
    parser.add_argument('--output-dir', default='plots', help='Directory for output plots')
    parser.add_argument('--show', action='store_true', help='Display plots interactively')
    
    args = parser.parse_args()
    
    generate_report(args.log_dir, args.output_dir)
    
    if args.show:
        plt.show()

if __name__ == '__main__':
    main()
