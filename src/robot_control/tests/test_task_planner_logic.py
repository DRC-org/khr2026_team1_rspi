import pytest
import sys
import os
from unittest.mock import MagicMock

# MOCK ROS 2 MODULES BEFORE IMPORT
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.duration"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["nav2_simple_commander"] = MagicMock()
sys.modules["nav2_simple_commander.robot_navigator"] = MagicMock()

# Add scripts directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../scripts'))

# Try importing
try:
    from task_planner import TaskOptimizer
except ImportError as e:
    pytest.skip(f"Failed to import task_planner: {e}", allow_module_level=True)

class TestTaskOptimizer:
    def setup_method(self):
        self.optimizer = TaskOptimizer()

    def test_distance_calculation(self):
        # Spot 0: (0.5, 0.5)
        # Spot 1: (0.5, 2.0)
        # Dist = 1.5
        dist = self.optimizer.calculate_distance(0, 1)
        assert dist == pytest.approx(1.5, 0.01)

    def test_solve_simple_case(self):
        # Solve with plenty of time
        # Should visit all V-Goal required spots (1, 2, 3, 5)
        # 1: Jintori_1
        # 2: Jintori_2
        # 3: Jintori_3
        # 5: Honmaru
        
        path = self.optimizer.solve(start_node_id=0, remaining_time=180)
        assert len(path) > 0
        
        v_goal_ids = [1, 2, 3, 5]
        visited = set(path)
        for vid in v_goal_ids:
            assert vid in visited, f"V-Goal spot {vid} not in path"
            
    def test_time_constraint(self):
        # Very short time
        path = self.optimizer.solve(start_node_id=0, remaining_time=1)
        # Should be empty or just start node
        # With 1s, it shouldn't be able to visit many spots
        assert len(path) <= 2  # Start + maybe one nearby spot
