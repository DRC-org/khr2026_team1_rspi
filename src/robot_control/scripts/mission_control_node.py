#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import json
import time

from robot_control.mission_logic.state import GameState, ZoneState
from robot_control.mission_logic.planner import ActionPlanner

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        
        self.state = GameState()
        self.planner = ActionPlanner()
        self.navigator = BasicNavigator()
        
        # Subscriptions
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(String, '/score_detail', self.score_callback, 10)
        self.create_subscription(String, '/robot_status', self.status_callback, 10)
        
        # Publishers
        self.control_pub = self.create_publisher(String, '/robot_control', 10)
        
        # Logic loop
        self.create_timer(1.0, self.control_loop)
        
        self.current_action = None
        self.is_executing_hardware = False
        self.last_hardware_cmd_time = 0
        
        self.get_logger().info("Mission Control (Robust Version) Initialized")

    def status_callback(self, msg):
        """Handle hardware feedback (Action Completed)"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"FEEDBACK: Module {hex(data['id'])} finished action.")
            # Clear flag to proceed
            self.is_executing_hardware = False
        except: pass

    def tf_callback(self, msg):
        """Update robot position from TF"""
        for transform in msg.transforms:
            if transform.child_frame_id == 'khr2026_robot':
                self.state.robot_x = transform.transform.translation.x
                self.state.robot_y = transform.transform.translation.y

    def score_callback(self, msg):
        """Update captured zone states from scoring node"""
        try:
            data = json.loads(msg.data)
            # In a real scenario, sync with score node state here
            pass 
        except: pass

    def control_loop(self):
        # 1. Check if moving or executing hardware
        if self.current_action:
            if not self.navigator.isTaskComplete():
                return # Still navigating to spot
            
            # Arrived! Execute hardware action if not started
            if not self.is_executing_hardware:
                if self.last_hardware_cmd_time == 0:
                    self.get_logger().info(f"Arrived at {self.current_action['name']}. Starting hardware sequence...")
                    self.execute_hardware_sequence(self.current_action)
                    self.is_executing_hardware = True
                    self.last_hardware_cmd_time = time.time()
                return

            # Wait for feedback or safety timeout (10s)
            if time.time() - self.last_hardware_cmd_time > 10.0:
                self.get_logger().warn("Hardware feedback timeout. Proceeding...")
                self.is_executing_hardware = False

            if self.is_executing_hardware:
                return # Still waiting for action completion

            # Action complete
            self.get_logger().info(f"Mission step at {self.current_action['name']} complete.")
            self.update_state_post_action(self.current_action)
            self.current_action = None
            self.is_executing_hardware = False
            self.last_hardware_cmd_time = 0
            
        # 2. Plan next move
        plan = self.planner.plan_mission(self.state)
        if not plan:
            return
            
        # 3. Trigger next navigation
        next_spot = plan[0]
        self.get_logger().info(f"Next Target: {next_spot['name']}")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = next_spot['x']
        goal_pose.pose.position.y = next_spot['y']
        goal_pose.pose.orientation.w = 1.0
        
        self.navigator.goToPose(goal_pose)
        self.current_action = next_spot

    def execute_hardware_sequence(self, action):
        """Send JSON commands to /robot_control based on spot type"""
        # Template for both mechanism sets
        control_data = {
            "yagura": {"1_pos": "stopped", "1_state": "open", "2_pos": "stopped", "2_state": "open"},
            "ring": {"1_pos": "stopped", "1_state": "open", "2_pos": "stopped", "2_state": "open"}
        }
        
        if action['type'] == 'supply':
            if action.get('gain_y', 0) > 0:
                control_data["yagura"]["1_pos"] = "down"
                control_data["yagura"]["1_state"] = "closed"
            if action.get('gain_r', 0) > 0:
                control_data["ring"]["1_pos"] = "pickup"
                control_data["ring"]["1_state"] = "open" # 把持(開)
        
        elif action['type'] == 'target':
            control_data["yagura"]["1_pos"] = "down"
            control_data["yagura"]["1_state"] = "open" # リリース(開)
            
        elif action['type'] == 'honmaru':
            control_data["ring"]["1_pos"] = "honmaru"
            control_data["ring"]["1_state"] = "closed" # リリース(閉)
            
        self.control_pub.publish(String(data=json.dumps(control_data)))

    def update_state_post_action(self, action):
        """Update internal state prediction after confirmed action"""
        if action['type'] == 'supply':
            self.state.held_yagura += action.get('gain_y', 0)
            self.state.held_rings += action.get('gain_r', 0)
        elif action['type'] == 'target':
            self.state.held_yagura -= action.get('req_y', 0)
            self.state.held_rings -= action.get('req_r', 0)
            zid = action.get('id')
            if zid:
                self.state.zones[zid].yagura += 1
                self.state.zones[zid].rings += 2
        elif action['type'] == 'honmaru':
            self.state.held_rings -= 1
            self.state.honmaru_rings += 1

def main():
    rclpy.init()
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
