#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time

class MinimalTest(Node):
    def __init__(self):
        super().__init__('minimal_test')
        self.scan_received = False
        self.odom_received = False
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
    
    def scan_cb(self, msg):
        self.scan_received = True
        
    def odom_cb(self, msg):
        self.odom_received = True

def main():
    rclpy.init()
    node = MinimalTest()
    start_time = time.time()
    timeout = 10.0
    
    print("Minimal Connectivity Test: Waiting for /scan and /odom...")
    while rclpy.ok() and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan_received and node.odom_received:
            break
            
    if node.scan_received and node.odom_received:
        print("[PASS] Both /scan and /odom received!")
        exit(0)
    else:
        if not node.scan_received: print("[FAIL] No /scan received")
        if not node.odom_received: print("[FAIL] No /odom received")
        exit(1)

if __name__ == '__main__':
    main()
