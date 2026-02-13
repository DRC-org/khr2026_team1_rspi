import pytest
import sys
import os
from unittest.mock import MagicMock

# MOCK ROS 2 MODULES
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["std_msgs"] = MagicMock()
sys.modules["std_msgs.msg"] = MagicMock()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

from robot_control.mission_logic.state import GameState, ZoneState
from robot_control.mission_logic.planner import ActionPlanner

class TestActionPlanner:
    def setup_method(self):
        try:
            self.planner = ActionPlanner()
        except ImportError:
            pytest.skip("OR-Tools not installed", allow_module_level=True)

    def test_plan_v_goal_scenario(self):
        """Test planning from start (Optimization)"""
        initial_state = GameState()
        
        # Plan actions
        actions = self.planner.plan_mission(initial_state)
        
        assert len(actions) > 0
        # First step should be supply since we have 0 items
        assert actions[0]['type'] == 'supply'

    def test_resource_starvation(self):
        """Planner must go to supply if it has no yagura"""
        state = GameState()
        state.held_yagura = 0
        state.held_rings = 4 # Full rings but no yagura
        
        actions = self.planner.plan_mission(state)
        assert len(actions) > 0
        assert actions[0]['type'] == 'supply'
        assert actions[0].get('gain_y', 0) > 0

    def test_ote_transition(self):
        """Once 3 zones are captured, go to Honmaru if rings are held"""
        state = GameState()
        # Capturing 3 zones
        for i in range(1, 4):
            state.zones[i] = ZoneState(yagura=1, rings=2)
        state.held_rings = 1 # Enough for V-Goal
        
        actions = self.planner.plan_mission(state)
        assert len(actions) > 0
        assert actions[0]['type'] == 'honmaru'

    def test_multi_step_sequence(self):
        """Test a multi-step sequence (Supply -> Target)"""
        state = GameState()
        state.held_yagura = 1
        state.held_rings = 2
        # Near Supply_R (0.7, 6.2)
        state.robot_x = 0.7
        state.robot_y = 6.0
        
        actions = self.planner.plan_mission(state)
        # Should decide to drop what it has or get more
        assert len(actions) > 0
        # Valid actions: go to Target or go to Supply_Y/R
        types = [a['type'] for a in actions]
        assert 'target' in types or 'supply' in types
