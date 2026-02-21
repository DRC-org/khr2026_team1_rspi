import os
import sys
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

@pytest.mark.launch_test
def generate_test_description():
    pkg_simulation = get_package_share_directory('khr2026_team1_simulation')
    launch_file_path = os.path.join(pkg_simulation, 'launch', 'minimal_test.launch.py')

    simulation_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )

    return launch.LaunchDescription([
        simulation_launch,
        launch_testing.actions.ReadyToTest()
    ])


class TestSimulationConnectivity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_sim_connectivity')
        self.scan_received = False
        self.odom_received = False

        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def scan_cb(self, msg):
        self.scan_received = True

    def odom_cb(self, msg):
        self.odom_received = True

    def test_sensors_publishing(self):
        """Test if /scan and /odom are published by the simulation bridge"""
        import time
        start_time = time.time()
        timeout = 15.0  # Give Gazebo time to start and render

        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.scan_received and self.odom_received:
                break

        print(f"\\nScan received: {self.scan_received}")
        print(f"Odom received: {self.odom_received}")
        
        self.assertTrue(self.scan_received, "Failed to receive /scan data from simulated LiDAR")
        self.assertTrue(self.odom_received, "Failed to receive /odom data from simulated robot")
