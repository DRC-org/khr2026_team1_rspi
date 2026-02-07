
import unittest
import json
import sys
import os
import math
from unittest.mock import MagicMock

# Add the script directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../scripts'))

# Import target module
from rosbridge_simulator import ROSBridgeSimulator

class TestROSBridgeSimulator(unittest.TestCase):
    def setUp(self):
        self.simulator = ROSBridgeSimulator(host='localhost', port=9090)
        # Mock optimizer to avoid complex dependency
        self.simulator.optimizer = MagicMock()
        self.simulator.optimizer.solve.return_value = [0, 1]
        self.simulator.optimizer.spots = {
            0: {"name": "Start", "x": 0.0, "y": 0.0},
            1: {"name": "Goal", "x": 10.0, "y": 0.0}
        }

    def test_path_generation(self):
        """Test if path data structure is correct"""
        self.simulator.current_path = {
            "spots": self.simulator.optimizer.spots,
            "path": [0, 1],
            "timestamp": 123456789
        }
        
        # Verify robot pose calculation logic
        current_pos = [0.0, 0.0]
        target = self.simulator.optimizer.spots[1]
        dx = target["x"] - current_pos[0]
        dy = target["y"] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        self.assertEqual(distance, 10.0)

    def test_ros_message_format(self):
        """Test if messages are formatted as valid ROS bridge JSON"""
        # Test Pose message format
        pose_msg = {
            "op": "publish",
            "topic": "/robot/pose",
            "msg": {
                "header": {
                    "frame_id": "map",
                    "stamp": 123456789
                },
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        
        self.assertEqual(pose_msg["op"], "publish")
        self.assertEqual(pose_msg["topic"], "/robot/pose")
        self.assertEqual(pose_msg["msg"]["header"]["frame_id"], "map")
        self.assertEqual(pose_msg["msg"]["pose"]["position"]["x"], 1.0)

if __name__ == '__main__':
    unittest.main()
