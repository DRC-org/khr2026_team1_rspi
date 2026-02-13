import pytest
import json
import sys
import os
from unittest.mock import MagicMock

# MOCK ROS 2
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["std_msgs"] = MagicMock()
sys.modules["std_msgs.msg"] = MagicMock()
sys.modules["nav_msgs"] = MagicMock()
sys.modules["nav_msgs.msg"] = MagicMock()
sys.modules["sensor_msgs"] = MagicMock()
sys.modules["sensor_msgs.msg"] = MagicMock()
sys.modules["tf2_ros"] = MagicMock()

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

from robot_control.mission_logic.manager import LogicManager
from robot_control.mission_logic.state import GameState

class TestLogicManager:
    def setup_method(self):
        self.manager = LogicManager()

    def test_strict_feedback_handling(self):
        """Verify that only relevant CAN IDs clear the hardware execution flag"""
        self.manager.prepare_action('target')
        assert self.manager.is_executing_hardware == True
        assert self.manager.waiting_for_feedback_id == 0x40 
        
        # 1. Send unrelated feedback (Battery 0x10)
        data_wrong = json.dumps({"id": 0x10, "battery_12v": 12.5})
        result = self.manager.process_feedback(data_wrong)
        
        assert result == False
        assert self.manager.is_executing_hardware == True
        
        # 2. Send correct feedback (0x40)
        data_right = json.dumps({"id": 0x40, "val": 1})
        result = self.manager.process_feedback(data_right)
        
        assert result == True
        assert self.manager.is_executing_hardware == False

    def test_multi_system_feedback(self):
        """Verify that feedback from Mech 1 doesn't clear flag for Mech 2"""
        # Start action for Mech 2 (Target action, assuming it's for mechanism index 2)
        # We need to update prepare_action to support an index
        self.manager.prepare_action('target', mech_index=2)
        
        # In our protocol (can.md):
        # Mech 1 Hand: 0x40, Mech 2 Hand: 0x41
        assert self.manager.waiting_for_feedback_id == 0x41
        
        # 1. Feedback from Mech 1 (Wrong)
        data_wrong = json.dumps({"id": 0x40, "val": 1})
        assert self.manager.process_feedback(data_wrong) == False
        assert self.manager.is_executing_hardware == True
        
        # 2. Feedback from Mech 2 (Correct)
        data_right = json.dumps({"id": 0x41, "val": 1})
        assert self.manager.process_feedback(data_right) == True
        assert self.manager.is_executing_hardware == False

    def test_state_synchronization(self):
        """Verify that GameState is updated from scoring_node messages"""
        assert self.manager.state.zones[1].yagura == 0
        
        # Receive scoring detail (Ground Truth)
        data = json.dumps({
            "total_score": 100,
            "zones": {
                "1": {"y": 1, "r": 2}
            }
        })
        self.manager.sync_state(data)
        
        # Internal state should sync
        assert self.manager.state.zones[1].yagura == 1
        assert self.manager.state.zones[1].rings == 2

    def test_nav_failure_recovery(self):
        """Logic for MissionControlNode (Manual logic check since it's a Node)"""
        # We can't easily test the Node class here due to ROS mocks, 
        # but we can define the expected logic behavior.
        # This is a placeholder to remind us to implement the check in mission_control_node.py
        pass
