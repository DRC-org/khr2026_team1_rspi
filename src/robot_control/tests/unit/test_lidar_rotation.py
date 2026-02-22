import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class RotationDiagnostic(Node):
    def __init__(self):
        super().__init__('rotation_diagnostic')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.last_scan = None
        self.scans_collected = []

    def scan_cb(self, msg):
        self.last_scan = np.array(msg.ranges)

    def run_test(self):
        print("Starting rotation test...")
        # 1. Capture initial scan
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.last_scan is None:
            print("No scan received")
            return
        
        initial_scan = self.last_scan.copy()
        
        # 2. Rotate
        print("Rotating robot...")
        msg = Twist()
        msg.angular.z = 0.5
        for _ in range(20):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop
        self.pub.publish(Twist())
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        final_scan = self.last_scan.copy()
        
        # 3. Compare
        # If the sensor is rotating, the values in the array should shift or change
        # (unless the environment is perfectly circular, which it isn't anymore with walls)
        diff = np.abs(initial_scan - final_scan)
        # Use only finite points for diff
        valid = np.isfinite(initial_scan) & np.isfinite(final_scan)
        mean_diff = np.mean(diff[valid]) if np.any(valid) else 0
        
        print(f"Mean difference in scan values after rotation: {mean_diff:.4f}")
        if mean_diff < 0.01:
            print("CRITICAL: Scan data did NOT change significantly during rotation!")
            print("The sensor might be fixed to the world frame.")
        else:
            print("Scan data changed. Sensor is likely rotating.")

def main():
    rclpy.init()
    node = RotationDiagnostic()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
