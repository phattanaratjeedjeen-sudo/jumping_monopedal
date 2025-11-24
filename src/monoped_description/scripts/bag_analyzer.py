#!/usr/bin/python3
"""
Bag file analyzer for hopping robot simulation.
Extracts data from ROS2 bag files and generates analysis plots.

Usage:
    python3 bag_analyzer.py <bag_path> [output_dir]

Example:
    python3 bag_analyzer.py ~/jumping_monopedal/bag_files/bag_1763963689
"""

import sys
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray


def extract_debug_state(bag_path: str) -> dict:
    """
    Extract /debug_state topic data from bag file.

    Data layout: [zb, zb_dot, zf, Hk, Hc, Hd, u, effort, state_code]
    state_code: air=0, compress=1, touchdown=2, rebound=3
    """
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")

    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = '/debug_state')
        ORDER BY timestamp
    """)
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        raise ValueError("No /debug_state messages found in bag file")

    timestamps = []
    zb_list, zb_dot_list, zf_list = [], [], []
    Hk_list, Hc_list, Hd_list = [], [], []
    u_list, effort_list, state_list = [], [], []

    for ts, data in rows:
        msg = deserialize_message(data, Float64MultiArray)
        if len(msg.data) >= 9:
            timestamps.append(ts / 1e9)
            zb_list.append(msg.data[0])
            zb_dot_list.append(msg.data[1])
            zf_list.append(msg.data[2])
            Hk_list.append(msg.data[3])
            Hc_list.append(msg.data[4])
            Hd_list.append(msg.data[5])
            u_list.append(msg.data[6])
            effort_list.append(msg.data[7])
            state_list.append(msg.data[8])

    t = np.array(timestamps)
    t = t - t[0]  # Start from 0

    return {
        't': t,
        'zb': np.array(zb_list),
        'zb_dot': np.array(zb_dot_list),
        'zf': np.array(zf_list),
        'Hk': np.array(Hk_list),
        'Hc': np.array(Hc_list),
        'Hd': np.array(Hd_list),
        'u': np.array(u_list),
        'effort': np.array(effort_list),
        'state': np.array(state_list),
    }


def find_apex_heights(zb: np.ndarray, zb_dot: np.ndarray, t: np.ndarray,
                      threshold: float = 0.01) -> tuple:
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
    print(f"\n{'='*50}")
    print("DATA STATISTICS")
    print(f"{'='*50}")
    print(f"Duration: {data['t'][-1]:.2f} seconds")
    print(f"\nBody height (zb):")
    print(f"  min: {np.nanmin(data['zb']):.4f} m")
    print(f"  max: {np.nanmax(data['zb']):.4f} m")
    print(f"\nBody velocity (zb_dot):")
    print(f"  min: {np.nanmin(data['zb_dot']):.4f} m/s")
    print(f"  max: {np.nanmax(data['zb_dot']):.4f} m/s")
    print(f"\nFoot height (zf):")
    print(f"  min: {np.nanmin(data['zf']):.4f} m")
    print(f"  max: {np.nanmax(data['zf']):.4f} m")
    print(f"\nDesired height (Hd): {data['Hd'][0]:.4f} m")
    print(f"\nEffort commands:")
    print(f"  min: {np.nanmin(data['effort']):.2f} N")
    print(f"  max: {np.nanmax(data['effort']):.2f} N")
    print(f"\n{'='*50}")
    print("APEX HEIGHT ANALYSIS")
    print(f"{'='*50}")
    print(f"Number of hops: {len(apex_heights)}")
    if len(apex_heights) > 0:
        print(f"Mean apex height: {np.mean(apex_heights):.4f} m")
        print(f"Std apex height: {np.std(apex_heights):.4f} m")
        print(f"First 5 apex heights: {apex_heights[:5]}")
        print(f"Last 5 apex heights: {apex_heights[-5:]}")


def plot_hopping_analysis(data: dict, apex_heights: np.ndarray,
                          output_path: str, time_window: float = 30.0):
    """
    Generate hopping analysis plot with 4 subplots.
    """
    t = data['t']
    mask = t < time_window

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=False)
    Hd = data['Hd'][0]

    # Plot 1: Body and foot height over time
    ax1 = axes[0]
    ax1.plot(t[mask], data['zb'][mask], 'b-', linewidth=0.5, label='Body height (zb)')
    ax1.plot(t[mask], data['zf'][mask], 'g-', linewidth=0.5, alpha=0.7, label='Foot height (zf)')
    ax1.axhline(y=Hd, color='r', linestyle='--', label=f'Desired Hd={Hd}m')
    ax1.set_ylabel('Height (m)')
    ax1.set_xlabel('Time (s)')
    ax1.set_title(f'Hopping Behavior (First {time_window:.0f} seconds)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Apex heights convergence
    ax2 = axes[1]
    ax2.plot(range(len(apex_heights)), apex_heights, 'bo-', markersize=3, linewidth=0.5)
    ax2.axhline(y=Hd, color='r', linestyle='--', label=f'Desired Hd={Hd}m')
    ax2.set_xlabel('Hop number')
    ax2.set_ylabel('Apex height (m)')
    ax2.set_title('Apex Height Convergence')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    if len(apex_heights) > 0:
        y_min = min(np.min(apex_heights), Hd) - 0.05
        y_max = max(np.max(apex_heights), Hd) + 0.05
        ax2.set_ylim([y_min, y_max])

    # Plot 3: Phase plot (zb vs zb_dot)
    ax3 = axes[2]
    ax3.plot(data['zb'][mask], data['zb_dot'][mask], 'b-', linewidth=0.3, alpha=0.7)
    ax3.set_xlabel('Body height zb (m)')
    ax3.set_ylabel('Body velocity (m/s)')
    ax3.set_title(f'Phase Plot (First {time_window:.0f} seconds)')
    ax3.grid(True, alpha=0.3)

    # Plot 4: Effort commands over time
    ax4 = axes[3]
    ax4.plot(t[mask], data['effort'][mask], 'r-', linewidth=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Effort (N)')
    ax4.set_title(f'Effort Commands (First {time_window:.0f} seconds)')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved: {output_path}")


def plot_poincare_map(apex_heights: np.ndarray, Hd: float, output_path: str):
    """
    Generate Poincaré map showing Hk vs Hk+1.
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    if len(apex_heights) > 1:
        ax.plot(apex_heights[:-1], apex_heights[1:], 'bo', markersize=3, alpha=0.5)

        # Plot diagonal line (Hk+1 = Hk)
        h_min = min(np.min(apex_heights), Hd) - 0.05
        h_max = max(np.max(apex_heights), Hd) + 0.05
        ax.plot([h_min, h_max], [h_min, h_max], 'k--', label='Hk+1 = Hk')

        # Plot desired height reference lines
        ax.axhline(y=Hd, color='r', linestyle=':', alpha=0.5, label=f'Hd={Hd}m')
        ax.axvline(x=Hd, color='r', linestyle=':', alpha=0.5)

        ax.set_xlabel('Current apex height Hk (m)')
        ax.set_ylabel('Next apex height Hk+1 (m)')
        ax.set_title('Poincaré Map')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim([h_min, h_max])
        ax.set_ylim([h_min, h_max])
        ax.set_aspect('equal')

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
    data = extract_debug_state(bag_path)

    # Find apex heights
    apex_heights, apex_times = find_apex_heights(
        data['zb'], data['zb_dot'], data['t']
    )

    # Print statistics
    print_statistics(data, apex_heights)

    # Generate plots
    plot_hopping_analysis(
        data, apex_heights,
        os.path.join(output_dir, 'hopping_analysis.png'),
        time_window=30.0
    )

    plot_poincare_map(
        apex_heights,
        data['Hd'][0],
        os.path.join(output_dir, 'poincare_map.png')
    )

    print(f"\nAnalysis complete!")


if __name__ == '__main__':
    main()
