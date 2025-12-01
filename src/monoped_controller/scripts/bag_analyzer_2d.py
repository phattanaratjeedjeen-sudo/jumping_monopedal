#!/usr/bin/python3
"""
Bag file analyzer for 2D hopping monoped robot simulation.
Extracts data from ROS2 bag files and generates analysis plots.

Usage:
    python3 bag_analyzer_2d.py <bag_path> [output_dir]

Example:
    python3 bag_analyzer_2d.py ~/jumping_monopedal/bag_files/bag_1764598703
"""

import sys
import os
import sqlite3
import struct
import numpy as np
import matplotlib.pyplot as plt


def parse_float64_multiarray(data):
    """Parse std_msgs/Float64MultiArray from raw bytes."""
    try:
        # For this bag format, data starts at offset 20
        if len(data) >= 68:
            values = struct.unpack_from('<6d', data, 20)
            return list(values)
        return None
    except:
        return None


def extract_debug_state(bag_path: str) -> dict:
    """
    Extract /debug_state topic data from bag file.

    Data layout for 2D monoped: [zb, zf, effort, theta, theta_dot, torque]
    """
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")

    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get topic id for /debug_state
    cursor.execute("SELECT id FROM topics WHERE name = '/debug_state'")
    result = cursor.fetchone()
    if not result:
        raise ValueError("No /debug_state topic found in bag file")
    topic_id = result[0]

    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (topic_id,))
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        raise ValueError("No /debug_state messages found in bag file")

    timestamps = []
    zb_list, zf_list, effort_list = [], [], []
    theta_list, theta_dot_list, torque_list = [], [], []

    for ts, data in rows:
        parsed = parse_float64_multiarray(data)
        if parsed and len(parsed) >= 6:
            timestamps.append(ts / 1e9)
            zb_list.append(parsed[0])
            zf_list.append(parsed[1])
            effort_list.append(parsed[2])
            theta_list.append(parsed[3])
            theta_dot_list.append(parsed[4])
            torque_list.append(parsed[5])

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


def compute_zb_dot(zb: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Compute velocity from position using finite differences."""
    zb_dot = np.gradient(zb, t)
    return zb_dot


def find_apex_heights(zb: np.ndarray, zb_dot: np.ndarray, t: np.ndarray,
                      threshold: float = 0.05) -> tuple:
    """
    Find apex heights where velocity crosses zero from positive to negative.
    """
    apex_heights = []
    apex_times = []

    for i in range(1, len(zb_dot)):
        if zb_dot[i-1] > threshold and zb_dot[i] <= threshold:
            apex_heights.append(zb[i])
            apex_times.append(t[i])

    return np.array(apex_heights), np.array(apex_times)


def print_statistics(data: dict, apex_heights: np.ndarray):
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
    
    print(f"\n{'='*60}")
    print("🦘 HOPPING ANALYSIS")
    print(f"{'='*60}")
    print(f"Number of hops detected: {len(apex_heights)}")
    if len(apex_heights) > 0:
        print(f"Mean apex height: {np.mean(apex_heights):.3f} m")
        print(f"Std apex height:  {np.std(apex_heights):.3f} m")
        print(f"Min apex height:  {np.min(apex_heights):.3f} m")
        print(f"Max apex height:  {np.max(apex_heights):.3f} m")
        if len(apex_heights) >= 5:
            print(f"First 5 apex heights: {[f'{h:.3f}' for h in apex_heights[:5]]}")
            print(f"Last 5 apex heights:  {[f'{h:.3f}' for h in apex_heights[-5:]]}")
    
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
    
    if len(apex_heights) > 5:
        print(f"  ✅ HOPPING! {len(apex_heights)} hops detected")
    elif len(apex_heights) > 0:
        print(f"  ⚠️  Few hops: {len(apex_heights)} hops detected")
    else:
        print(f"  ❌ NO HOPPING detected")


def plot_hopping_analysis(data: dict, apex_heights: np.ndarray, apex_times: np.ndarray,
                          output_path: str, time_window: float = None):
    """
    Generate hopping analysis plot with 6 subplots.
    """
    t = data['t']
    if time_window is None:
        time_window = t[-1]
    mask = t <= time_window

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'2D Monoped Hopping Analysis', fontsize=14, fontweight='bold')

    # Plot 1: Theta (pitch angle)
    ax = axes[0, 0]
    ax.plot(t[mask], data['theta'][mask], 'b-', linewidth=0.5)
    ax.axhline(y=90, color='g', linestyle='--', linewidth=2, label='Target (90°)')
    ax.fill_between(t[mask], 85, 95, alpha=0.2, color='green', label='±5° band')
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
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('Reaction Wheel Torque')
    ax.grid(True, alpha=0.3)

    # Plot 4: Apex heights convergence
    ax = axes[1, 1]
    if len(apex_heights) > 0:
        ax.plot(range(len(apex_heights)), apex_heights, 'bo-', markersize=4, linewidth=1)
        ax.axhline(y=np.mean(apex_heights), color='r', linestyle='--', 
                   label=f'Mean={np.mean(apex_heights):.3f}m')
        ax.set_xlabel('Hop number')
        ax.set_ylabel('Apex height (m)')
        ax.legend(loc='upper right')
    else:
        ax.text(0.5, 0.5, 'No hops detected', ha='center', va='center', transform=ax.transAxes)
    ax.set_title('Apex Height per Hop')
    ax.grid(True, alpha=0.3)

    # Plot 5: Effort
    ax = axes[2, 0]
    ax.plot(t[mask], data['effort'][mask], 'g-', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Effort (N)')
    ax.set_title('Spring Effort Command')
    ax.grid(True, alpha=0.3)

    # Plot 6: Phase plot (theta vs theta_dot)
    ax = axes[2, 1]
    colors = plt.cm.viridis(np.linspace(0, 1, np.sum(mask)))
    ax.scatter(data['theta'][mask], data['theta_dot'][mask], c=colors, s=1, alpha=0.5)
    ax.axvline(x=90, color='g', linestyle='--', linewidth=2, alpha=0.7)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Theta (deg)')
    ax.set_ylabel('Theta_dot (deg/s)')
    ax.set_title('Phase Plot (color = time)')
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

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f'Zoomed View: {time_start:.1f}s - {time_end:.1f}s', fontsize=14, fontweight='bold')

    # Theta zoomed
    ax = axes[0, 0]
    ax.plot(t[mask], data['theta'][mask], 'b-', linewidth=1.5)
    ax.axhline(y=90, color='g', linestyle='--', linewidth=2, label='Target (90°)')
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

    # Compute velocity from position
    zb_dot = compute_zb_dot(data['zb'], data['t'])

    # Find apex heights
    apex_heights, apex_times = find_apex_heights(data['zb'], zb_dot, data['t'])

    # Print statistics
    print_statistics(data, apex_heights)

    # Generate plots
    print(f"\n📊 Generating plots...")
    
    plot_hopping_analysis(
        data, apex_heights, apex_times,
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
