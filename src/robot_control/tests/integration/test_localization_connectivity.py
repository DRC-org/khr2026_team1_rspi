#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class LocalizationTest(Node):
    def __init__(self):
        super().__init__('localization_test')
        self.scan_received = False
        self.pose_received = False
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        
    def scan_cb(self, msg):
        self.scan_received = True
        
    def pose_cb(self, msg):
        self.pose_received = True

def main():
    rclpy.init()
    node = LocalizationTest()
    
    start_time = time.time()
    timeout = 10.0
    
    print("Running localization connectivity test...")
    while rclpy.ok() and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan_received and node.pose_received:
            break
            
    success = True
    if not node.scan_received:
        print("[FAIL] No LaserScan data received on /scan")
        success = False
    else:
        print("[PASS] LaserScan data received")
        
    if not node.pose_received:
        print("[FAIL] No Pose data received on /amcl_pose (AMCL not localizing)")
        success = False
    else:
        print("[PASS] AMCL Pose received")
        
    if success:
        print("\nOVERALL: SUCCESS")
        exit(0)
    else:
        print("\nOVERALL: FAIL")
        exit(1)

if __name__ == '__main__':
    main()
