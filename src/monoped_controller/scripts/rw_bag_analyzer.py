#!/usr/bin/python3
"""
Reaction Wheel Stabilizer Bag Analyzer
Extracts and visualizes RW stabilization data from ROS2 bag files.

Usage:
    python3 rw_bag_analyzer.py <bag_path> [output_dir]

Example:
    python3 rw_bag_analyzer.py ~/jumping_monopedal/bag_files/bag_1763977160
"""

import sys
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import math


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw


def extract_rw_debug(bag_path: str) -> dict:
    """
    Extract /rw_debug topic data from bag file.
    
    Data layout: [pitch, derivative, vel_cmd, spring_pos, tau_spring, P, I, D]
    """
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")
    
    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = '/rw_debug')
        ORDER BY timestamp
    """)
    rows = cursor.fetchall()
    conn.close()
    
    if not rows:
        raise ValueError("No /rw_debug messages found in bag file")
    
    timestamps = []
    pitch_list, deriv_list, vel_cmd_list = [], [], []
    spring_pos_list, tau_spring_list = [], []
    P_list, I_list, D_list = [], [], []
    
    for ts, data in rows:
        msg = deserialize_message(data, Float64MultiArray)
        if len(msg.data) >= 8:
            timestamps.append(ts / 1e9)
            pitch_list.append(msg.data[0])
            deriv_list.append(msg.data[1])
            vel_cmd_list.append(msg.data[2])
            spring_pos_list.append(msg.data[3])
            tau_spring_list.append(msg.data[4])
            P_list.append(msg.data[5])
            I_list.append(msg.data[6])
            D_list.append(msg.data[7])
    
    t = np.array(timestamps)
    t = t - t[0]  # Start from 0
    
    return {
        't': t,
        'pitch': np.array(pitch_list),
        'derivative': np.array(deriv_list),
        'vel_cmd': np.array(vel_cmd_list),
        'spring_pos': np.array(spring_pos_list),
        'tau_spring': np.array(tau_spring_list),
        'P': np.array(P_list),
        'I': np.array(I_list),
        'D': np.array(D_list),
    }


def extract_imu_data(bag_path: str) -> dict:
    """Extract IMU data for cross-validation."""
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")
    
    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = '/imu_data')
        ORDER BY timestamp
    """)
    rows = cursor.fetchall()
    conn.close()
    
    if not rows:
        return None
    
    timestamps = []
    pitch_list, pitch_rate_list = [], []
    
    for ts, data in rows:
        msg = deserialize_message(data, Imu)
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        timestamps.append(ts / 1e9)
        pitch_list.append(pitch)
        pitch_rate_list.append(msg.angular_velocity.y)
    
    t = np.array(timestamps)
    t = t - t[0]
    
    return {
        't': t,
        'pitch': np.array(pitch_list),
        'pitch_rate': np.array(pitch_rate_list),
    }


def extract_velocity_commands(bag_path: str) -> dict:
    """Extract velocity controller commands."""
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")
    
    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = '/velocity_controller/commands')
        ORDER BY timestamp
    """)
    rows = cursor.fetchall()
    conn.close()
    
    if not rows:
        return None
    
    timestamps = []
    left_list, right_list = [], []
    
    for ts, data in rows:
        msg = deserialize_message(data, Float64MultiArray)
        if len(msg.data) >= 2:
            timestamps.append(ts / 1e9)
            left_list.append(msg.data[0])
            right_list.append(msg.data[1])
    
    t = np.array(timestamps)
    t = t - t[0]
    
    return {
        't': t,
        'left': np.array(left_list),
        'right': np.array(right_list),
    }


def print_statistics(data: dict, imu_data: dict = None, vel_data: dict = None):
    """Print data statistics to console."""
    print(f"\n{'='*60}")
    print("REACTION WHEEL STABILIZER STATISTICS")
    print(f"{'='*60}")
    print(f"Duration: {data['t'][-1]:.2f} seconds")
    print(f"Number of control updates: {len(data['t'])}")
    
    print(f"\nPitch Angle:")
    print(f"  min: {np.rad2deg(np.min(data['pitch'])):7.2f}° ({np.min(data['pitch']):.4f} rad)")
    print(f"  max: {np.rad2deg(np.max(data['pitch'])):7.2f}° ({np.max(data['pitch']):.4f} rad)")
    print(f"  mean: {np.rad2deg(np.mean(data['pitch'])):7.2f}° ({np.mean(data['pitch']):.4f} rad)")
    print(f"  std: {np.rad2deg(np.std(data['pitch'])):7.2f}° ({np.std(data['pitch']):.4f} rad)")
    
    print(f"\nPitch Rate (derivative):")
    print(f"  min: {np.min(data['derivative']):8.4f} rad/s")
    print(f"  max: {np.max(data['derivative']):8.4f} rad/s")
    print(f"  mean: {np.mean(data['derivative']):8.4f} rad/s")
    
    print(f"\nReaction Wheel Velocity Commands:")
    print(f"  min: {np.min(data['vel_cmd']):8.2f} rad/s")
    print(f"  max: {np.max(data['vel_cmd']):8.2f} rad/s")
    print(f"  mean: {np.mean(data['vel_cmd']):8.2f} rad/s")
    print(f"  std: {np.std(data['vel_cmd']):8.2f} rad/s")
    
    print(f"\nPID Terms:")
    print(f"  P: min={np.min(data['P']):8.2f}, max={np.max(data['P']):8.2f}, mean={np.mean(data['P']):8.2f}")
    print(f"  I: min={np.min(data['I']):8.2f}, max={np.max(data['I']):8.2f}, mean={np.mean(data['I']):8.2f}")
    print(f"  D: min={np.min(data['D']):8.2f}, max={np.max(data['D']):8.2f}, mean={np.mean(data['D']):8.2f}")
    
    print(f"\nSpring Joint:")
    print(f"  Position: min={np.min(data['spring_pos']):.4f} m, max={np.max(data['spring_pos']):.4f} m")
    print(f"  Effort: min={np.min(data['tau_spring']):.2f} N, max={np.max(data['tau_spring']):.2f} N")
    
    if vel_data is not None:
        print(f"\n{'='*60}")
        print("VELOCITY CONTROLLER COMMANDS")
        print(f"{'='*60}")
        print(f"Number of commands: {len(vel_data['t'])}")
        print(f"Left wheel:  min={np.min(vel_data['left']):8.2f}, max={np.max(vel_data['left']):8.2f} rad/s")
        print(f"Right wheel: min={np.min(vel_data['right']):8.2f}, max={np.max(vel_data['right']):8.2f} rad/s")
    else:
        print(f"\nWARNING: No velocity controller commands found!")
        print(f"         Reaction wheels may not be receiving commands.")
    
    if imu_data is not None:
        print(f"\n{'='*60}")
        print("IMU DATA (Cross-validation)")
        print(f"{'='*60}")
        print(f"Pitch from IMU: min={np.rad2deg(np.min(imu_data['pitch'])):7.2f}°, "
              f"max={np.rad2deg(np.max(imu_data['pitch'])):7.2f}°")
        print(f"Pitch rate: min={np.min(imu_data['pitch_rate']):8.4f}, "
              f"max={np.max(imu_data['pitch_rate']):8.4f} rad/s")


def plot_stabilization_analysis(data: dict, imu_data: dict, vel_data: dict,
                                output_path: str, time_window: float = 30.0):
    """Generate comprehensive stabilization analysis plot."""
    t = data['t']
    mask = t < time_window
    
    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
    
    # Plot 1: Pitch angle
    ax1 = axes[0]
    ax1.plot(t[mask], np.rad2deg(data['pitch'][mask]), 'b-', linewidth=1, label='Pitch (controller)')
    if imu_data is not None:
        imu_mask = imu_data['t'] < time_window
        ax1.plot(imu_data['t'][imu_mask], np.rad2deg(imu_data['pitch'][imu_mask]), 
                'r--', linewidth=0.5, alpha=0.7, label='Pitch (IMU)')
    ax1.axhline(y=0, color='k', linestyle=':', linewidth=1)
    ax1.set_ylabel('Pitch Angle (°)')
    ax1.set_title(f'Reaction Wheel Stabilization Analysis (First {time_window:.0f} seconds)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Reaction wheel velocity commands
    ax2 = axes[1]
    ax2.plot(t[mask], data['vel_cmd'][mask], 'g-', linewidth=1, label='Commanded velocity')
    if vel_data is not None:
        vel_mask = vel_data['t'] < time_window
        ax2.plot(vel_data['t'][vel_mask], vel_data['left'][vel_mask], 
                'c--', linewidth=0.5, alpha=0.7, label='Left wheel (actual)')
        ax2.plot(vel_data['t'][vel_mask], vel_data['right'][vel_mask], 
                'm--', linewidth=0.5, alpha=0.7, label='Right wheel (actual)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: PID terms
    ax3 = axes[2]
    ax3.plot(t[mask], data['P'][mask], 'r-', linewidth=0.8, label='P term')
    ax3.plot(t[mask], data['I'][mask], 'g-', linewidth=0.8, label='I term')
    ax3.plot(t[mask], data['D'][mask], 'b-', linewidth=0.8, label='D term')
    ax3.set_ylabel('PID Terms')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Pitch derivative
    ax4 = axes[3]
    ax4.plot(t[mask], data['derivative'][mask], 'purple', linewidth=1, label='Pitch rate (derivative)')
    if imu_data is not None:
        imu_mask = imu_data['t'] < time_window
        ax4.plot(imu_data['t'][imu_mask], imu_data['pitch_rate'][imu_mask], 
                'orange', linewidth=0.5, alpha=0.7, label='Pitch rate (IMU)')
    ax4.set_ylabel('Pitch Rate (rad/s)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Spring joint
    ax5 = axes[4]
    ax5_twin = ax5.twinx()
    l1 = ax5.plot(t[mask], data['spring_pos'][mask], 'b-', linewidth=1, label='Spring position')
    l2 = ax5_twin.plot(t[mask], data['tau_spring'][mask], 'r-', linewidth=1, label='Spring effort')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Position (m)', color='b')
    ax5_twin.set_ylabel('Effort (N)', color='r')
    ax5.tick_params(axis='y', labelcolor='b')
    ax5_twin.tick_params(axis='y', labelcolor='r')
    
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax5.legend(lines, labels, loc='upper right')
    ax5.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved: {output_path}")


def plot_phase_portrait(data: dict, output_path: str):
    """Generate pitch phase portrait (pitch vs pitch_rate)."""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    scatter = ax.scatter(np.rad2deg(data['pitch']), data['derivative'], 
                        c=data['t'], cmap='viridis', s=1, alpha=0.5)
    ax.set_xlabel('Pitch Angle (°)')
    ax.set_ylabel('Pitch Rate (rad/s)')
    ax.set_title('Phase Portrait: Pitch vs Pitch Rate')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle=':', linewidth=1)
    ax.axvline(x=0, color='k', linestyle=':', linewidth=1)
    
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Time (s)')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved: {output_path}")


def plot_control_effectiveness(data: dict, output_path: str, time_window: float = 10.0):
    """Plot control effectiveness over a shorter time window."""
    t = data['t']
    mask = t < time_window
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # Plot 1: Error and command
    ax1 = axes[0]
    ax1.plot(t[mask], np.rad2deg(data['pitch'][mask]), 'b-', linewidth=1.5, label='Pitch error')
    ax1.axhline(y=0, color='r', linestyle='--', linewidth=1, label='Desired (0°)')
    ax1.set_ylabel('Pitch (°)')
    ax1.set_title(f'Control Response (First {time_window:.0f} seconds)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Control output
    ax2 = axes[1]
    ax2.plot(t[mask], data['vel_cmd'][mask], 'g-', linewidth=1.5)
    ax2.set_ylabel('RW Velocity Cmd (rad/s)')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: PID contributions
    ax3 = axes[2]
    total = data['P'][mask] + data['I'][mask] + data['D'][mask]
    ax3.plot(t[mask], data['P'][mask], 'r-', linewidth=1, alpha=0.7, label='P')
    ax3.plot(t[mask], data['I'][mask], 'g-', linewidth=1, alpha=0.7, label='I')
    ax3.plot(t[mask], data['D'][mask], 'b-', linewidth=1, alpha=0.7, label='D')
    ax3.plot(t[mask], total, 'k-', linewidth=2, label='Total (P+I+D)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('PID Terms')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved: {output_path}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    bag_path = os.path.expanduser(sys.argv[1])
    output_dir = sys.argv[2] if len(sys.argv) > 2 else bag_path
    output_dir = os.path.expanduser(output_dir)
    
    if not os.path.exists(bag_path):
        print(f"Error: Bag path not found: {bag_path}")
        sys.exit(1)
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Analyzing bag: {bag_path}")
    print(f"Output directory: {output_dir}")
    
    # Extract data
    print("\nExtracting /rw_debug data...")
    data = extract_rw_debug(bag_path)
    
    print("Extracting /imu_data...")
    imu_data = extract_imu_data(bag_path)
    
    print("Extracting /velocity_controller/commands...")
    vel_data = extract_velocity_commands(bag_path)
    
    # Print statistics
    print_statistics(data, imu_data, vel_data)
    
    # Generate plots
    print(f"\n{'='*60}")
    print("GENERATING PLOTS")
    print(f"{'='*60}")
    
    plot_stabilization_analysis(
        data, imu_data, vel_data,
        os.path.join(output_dir, 'rw_stabilization_analysis.png'),
        time_window=30.0
    )
    
    plot_phase_portrait(
        data,
        os.path.join(output_dir, 'rw_phase_portrait.png')
    )
    
    plot_control_effectiveness(
        data,
        os.path.join(output_dir, 'rw_control_effectiveness.png'),
        time_window=10.0
    )
    
    print(f"\nAnalysis complete!")


if __name__ == '__main__':
    main()
