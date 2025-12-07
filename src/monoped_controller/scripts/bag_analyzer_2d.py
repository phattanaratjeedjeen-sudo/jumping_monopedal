#!/usr/bin/python3


import sys
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from monoped_interfaces.msg import Debug2D


def extract_debug_state(bag_path: str) -> dict:
    """
    Extract /debug_2d topic data from bag file.

    Uses Debug2D message type with fields:
    zb, zf, effort, theta, theta_dot, torque
    """
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")

    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = '/debug_2d')
        ORDER BY timestamp
    """)
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        raise ValueError("No /debug_2d messages found in bag file")

    timestamps = []
    zb_list, zf_list, effort_list = [], [], []
    theta_list, theta_dot_list, torque_list = [], [], []

    for ts, data in rows:
        msg = deserialize_message(data, Debug2D)
        timestamps.append(ts / 1e9)
        zb_list.append(msg.zb)
        zf_list.append(msg.zf)
        effort_list.append(msg.force)
        theta_list.append(msg.theta)
        theta_dot_list.append(msg.theta_dot)
        torque_list.append(msg.torque)

    t = np.array(timestamps)
    t = t - t[0]  # Start from 0

    return {
        't': t,
        'zb': np.array(zb_list),
        'zf': np.array(zf_list),
        'effort': np.array(effort_list),
        'theta': np.array(theta_list),
        'theta_dot': np.array(theta_dot_list),
        'torque': np.array(torque_list),
    }


def print_statistics(data: dict):
    """Print data statistics to console."""
    print(f"\n{'='*60}")
    print("📊 DATA STATISTICS")
    print(f"{'='*60}")
    print(f"Duration: {data['t'][-1]:.2f} seconds")
    print(f"Samples: {len(data['t'])}")
    
    print(f"\n📏 HEIGHTS:")
    print(f"  Body height (zb): {np.nanmin(data['zb']):.3f} - {np.nanmax(data['zb']):.3f} m")
    print(f"  Foot height (zf): {np.nanmin(data['zf']):.3f} - {np.nanmax(data['zf']):.3f} m")
    
    print(f"\n🎯 ATTITUDE CONTROL:")
    print(f"  Theta:     {np.nanmin(data['theta']):.2f}° - {np.nanmax(data['theta']):.2f}° (target: 90°)")
    print(f"  Mean:      {np.nanmean(data['theta']):.2f}°")
    print(f"  Std:       {np.nanstd(data['theta']):.2f}°")
    print(f"  Theta_dot: {np.nanmin(data['theta_dot']):.2f} - {np.nanmax(data['theta_dot']):.2f} °/s")
    print(f"  Torque:    {np.nanmin(data['torque']):.2f} - {np.nanmax(data['torque']):.2f} Nm")
    
    print(f"\n⚡ EFFORT:")
    print(f"  Spring effort: {np.nanmin(data['effort']):.2f} - {np.nanmax(data['effort']):.2f} N")
    
    # Diagnosis
    print(f"\n{'='*60}")
    print("🔍 DIAGNOSIS")
    print(f"{'='*60}")
    
    final_theta = data['theta'][-1]
    if abs(final_theta - 90) < 5:
        print(f"  ✅ BALANCED! Final theta = {final_theta:.2f}°")
    elif abs(final_theta - 90) < 15:
        print(f"  ⚠️  TILTED. Final theta = {final_theta:.2f}°")
    else:
        print(f"  ❌ FELL! Final theta = {final_theta:.2f}°")


def plot_hopping_analysis(data: dict, output_path: str, time_window: float = 30.0):
    """
    Generate hopping analysis plot with 4 subplots.
    """
    t = data['t']
    if time_window is None:
        time_window = t[-1]
    mask = t <= time_window

    fig, axes = plt.subplots(2, 2, figsize=(14, 7))
    fig.suptitle(f'2D Monoped Analysis (First {time_window:.0f} seconds)', fontsize=14, fontweight='bold')

    # Plot 1: Theta (pitch angle)
    ax = axes[0, 0]
    ax.plot(t[mask], data['theta'][mask], 'b-', linewidth=0.5)
    ax.axhline(y=95, color='orange', linestyle='--', linewidth=1.5, alpha=0.7, label='Forward Target (95°)')
    ax.axhline(y=85, color='purple', linestyle='--', linewidth=1.5, alpha=0.7, label='Backward Target (85°)')
    ax.fill_between(t[mask], 85, 95, alpha=0.2, color='gray', label='Target Range')
    ax.set_ylabel('Theta (deg)')
    ax.set_title('Body Pitch Angle')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Plot 2: Body and foot height
    ax = axes[0, 1]
    ax.plot(t[mask], data['zb'][mask], 'b-', linewidth=0.5, label='Body (zb)')
    ax.plot(t[mask], data['zf'][mask], 'orange', linewidth=0.5, label='Foot (zf)')
    ax.axhline(y=0, color='k', linestyle='-', linewidth=1, label='Ground')
    ax.set_ylabel('Height (m)')
    ax.set_title('Body & Foot Height')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Plot 3: Torque
    ax = axes[1, 0]
    ax.plot(t[mask], data['torque'][mask], 'm-', linewidth=0.5)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('Reaction Wheel Torque')
    ax.grid(True, alpha=0.3)

    # Plot 4: Effort
    ax = axes[1, 1]
    ax.plot(t[mask], data['effort'][mask], 'g-', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Effort (N)')
    ax.set_title('Spring Effort Command')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  ✅ Saved: {output_path}")


def plot_zoomed(data: dict, output_path: str, time_start: float = 0, time_end: float = 10):
    """
    Generate zoomed plot for detailed view.
    """
    t = data['t']
    mask = (t >= time_start) & (t <= time_end)

    fig, axes = plt.subplots(2, 2, figsize=(14, 7))
    fig.suptitle(f'Zoomed View: {time_start:.1f}s - {time_end:.1f}s', fontsize=14, fontweight='bold')

    # Theta zoomed
    ax = axes[0, 0]
    ax.plot(t[mask], data['theta'][mask], 'b-', linewidth=1.5)
    ax.axhline(y=95, color='orange', linestyle='--', linewidth=1.5, alpha=0.7, label='Forward Target (95°)')
    ax.axhline(y=85, color='purple', linestyle='--', linewidth=1.5, alpha=0.7, label='Backward Target (85°)')
    ax.fill_between(t[mask], 85, 95, alpha=0.2, color='gray')
    ax.set_ylabel('Theta (deg)')
    ax.set_title('Body Pitch Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Heights zoomed
    ax = axes[0, 1]
    ax.plot(t[mask], data['zb'][mask], 'b-', linewidth=1.5, label='Body (zb)')
    ax.plot(t[mask], data['zf'][mask], 'orange', linewidth=1.5, label='Foot (zf)')
    ax.axhline(y=0, color='k', linestyle='-', linewidth=1, label='Ground')
    ax.set_ylabel('Height (m)')
    ax.set_title('Body & Foot Height')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Torque zoomed
    ax = axes[1, 0]
    ax.plot(t[mask], data['torque'][mask], 'm-', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('Reaction Wheel Torque')
    ax.grid(True, alpha=0.3)

    # Effort zoomed
    ax = axes[1, 1]
    ax.plot(t[mask], data['effort'][mask], 'g-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Effort (N)')
    ax.set_title('Spring Effort Command')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  ✅ Saved: {output_path}")


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

    print(f"\n{'='*60}")
    print("🤖 2D MONOPED BAG ANALYZER")
    print(f"{'='*60}")
    print(f"📁 Bag path: {bag_path}")
    print(f"📂 Output:   {output_dir}")

    # Extract data
    print("\n📥 Extracting data...")
    data = extract_debug_state(bag_path)
    print(f"   Found {len(data['t'])} samples over {data['t'][-1]:.2f} seconds")

    # Print statistics
    print_statistics(data)

    # Generate plots
    print(f"\n📊 Generating plots...")
    
    plot_hopping_analysis(
        data,
        os.path.join(output_dir, 'hopping_analysis.png'),
        time_window=30.0  # First 30 seconds
    )

    plot_zoomed(
        data,
        os.path.join(output_dir, 'zoomed_0_10s.png'),
        time_start=0, time_end=10
    )

    print(f"\n{'='*60}")
    print("✅ Analysis complete!")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
