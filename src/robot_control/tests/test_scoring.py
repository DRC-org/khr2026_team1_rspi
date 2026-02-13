import pytest
import sys
import os
from unittest.mock import MagicMock

# MOCK ROS 2
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

from robot_control.scoring import ScoringManager

class TestScoringManager:
    def test_initial_score(self):
        sm = ScoringManager(is_auto=True)
        assert sm.get_total_score() == 0

    def test_ote_auto(self):
        sm = ScoringManager(is_auto=True)
        # 3 Yagura with 2 rings each
        for i in range(1, 4):
            sm.update_zone_state(zone_id=i, yagura_count=1, rings_in_yagura=2, rings_on_floor=0)
        
        # (10 + 40 + 30) * 3 + 100 = 340
        assert sm.get_total_score() == 340
        assert sm.is_ote() == True

    def test_v_goal_auto(self):
        sm = ScoringManager(is_auto=True)
        for i in range(1, 4):
            sm.update_zone_state(zone_id=i, yagura_count=1, rings_in_yagura=2, rings_on_floor=0)
        
        sm.update_honmaru_state(rings_in_honmaru=1)
        assert sm.is_v_goal() == True
        # 340 + 50 (honmaru ring) = 390
        assert sm.get_total_score() == 390
