import pytest
import sys
import os
from unittest.mock import MagicMock

# MOCK ROS 2 MODULES BEFORE IMPORT
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["std_msgs"] = MagicMock()
sys.modules["std_msgs.msg"] = MagicMock()

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

from robot_control.mission_logic.state import GameState, ZoneState
from robot_control.mission_logic.rules import RuleManager

class TestRuleManager:
    def setup_method(self):
        self.rules = RuleManager(is_auto=True)

    def test_capacity_limits(self):
        """Test hardware holding capacity limits"""
        state = GameState()
        
        # Initial state
        assert self.rules.can_pick_yagura(state, amount=1) == True
        
        # Max capacity check (Assumed Max Yagura = 2)
        state.held_yagura = 2
        assert self.rules.can_pick_yagura(state, amount=1) == False
        
        # Max capacity check (Assumed Max Ring = 4)
        state.held_rings = 4
        assert self.rules.can_pick_ring(state, amount=1) == False

    def test_area_entry_restrictions(self):
        """Test entry restrictions based on held items (Rulebook Area 2/3)"""
        state = GameState()
        
        # Auto Robot: Entry to Area 2 allowed if Rings <= 5
        # Our hardware limit is 4, so this is physically always true, 
        # but let's test the logic boundary in case hardware changes.
        state.held_rings = 6 
        assert self.rules.can_enter_area_2(state) == False
        
        state.held_rings = 5
        assert self.rules.can_enter_area_2(state) == True

    def test_ote_condition(self):
        """Test Ote (Checkmate) detection logic"""
        state = GameState()
        
        # 3 zones, each with 1 yagura and 2 rings -> Ote for Auto Robot
        state.zones[1] = ZoneState(yagura=1, rings=2)
        state.zones[2] = ZoneState(yagura=1, rings=2)
        state.zones[3] = ZoneState(yagura=1, rings=2)
        
        assert self.rules.is_ote(state) == True
        
        # Insufficient rings
        state.zones[3].rings = 1
        assert self.rules.is_ote(state) == False

    def test_v_goal_condition(self):
        """Test V-Goal (Conquest) condition"""
        state = GameState()
        
        # Ote setup
        for i in range(1, 4):
            state.zones[i] = ZoneState(yagura=1, rings=2)
            
        assert self.rules.is_ote(state) == True
        assert self.rules.is_v_goal(state) == False
        
        # Ring in Honmaru
        state.honmaru_rings = 1
        assert self.rules.is_v_goal(state) == True
        
        # Honmaru ring WITHOUT Ote
        state.zones[1].rings = 0 # Break Ote
        assert self.rules.is_ote(state) == False
        assert self.rules.is_v_goal(state) == False
