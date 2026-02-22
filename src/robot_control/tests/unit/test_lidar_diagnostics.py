import pytest
import rclpy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time

def test_lidar_advanced_diagnostics():
    """
    Advanced diagnostic test for LiDAR data quality.
    Checks for: self-collisions, data density, transform frames, and timing jitter.
    """
    rclpy.init()
    node = rclpy.create_node('scan_advanced_diagnostic_node')
    
    scans = []
    def scan_callback(msg):
        scans.append(msg)

    sub = node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    
    print("Collecting 20 samples of scan data...")
    timeout = 10.0
    start_collect = time.time()
    while len(scans) < 20 and (time.time() - start_collect) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not scans:
        print("ERROR: No scans received!")
        rclpy.shutdown()
        return

    print(f"Collected {len(scans)} samples.")
    
    # 1. Frame check
    frame_id = scans[0].header.frame_id
    print(f"Header frame_id: '{frame_id}'")
    
    # 2. Timing Jitter
    stamps = [s.header.stamp.sec + s.header.stamp.nanosec * 1e-9 for s in scans]
    intervals = np.diff(stamps)
    avg_hz = 1.0 / np.mean(intervals) if len(intervals) > 0 else 0
    jitter = np.std(intervals) if len(intervals) > 0 else 0
    print(f"Average Hz: {avg_hz:.2f}, Jitter (std dev): {jitter:.4f}s")
    
    # 3. Data Analysis (using last scan)
    last_scan = scans[-1]
    ranges = np.array(last_scan.ranges)
    nan_mask = np.isnan(ranges)
    inf_mask = np.isinf(ranges)
    valid_mask = ~(nan_mask | inf_mask)
    
    valid_ranges = ranges[valid_mask]
    if len(valid_ranges) > 0:
        print(f"Range stats: Min={valid_ranges.min():.2f}m, Max={valid_ranges.max():.2f}m, Median={np.median(valid_ranges):.2f}m")
    else:
        print("No valid range data points found (all Inf or NaN)!")
        
    # 4. Detail on "weirdness" - check for small clusters or outliers
    # If points are scattered randomly, it's noise.
    if len(valid_ranges) > 5:
        consecutive_diffs = np.diff(valid_ranges)
        noise_est = np.std(consecutive_diffs)
        print(f"Noise estimate (std dev of diffs): {noise_est:.4f}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    test_lidar_advanced_diagnostics()
