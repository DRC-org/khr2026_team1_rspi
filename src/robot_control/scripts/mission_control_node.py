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
from robot_control.mission_logic.manager import LogicManager

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        
        self.state = GameState()
        self.logic = LogicManager(self.state)
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
        self.last_hardware_cmd_time = 0
        
        self.get_logger().info("Mission Control (Production Version) Initialized")

    def status_callback(self, msg):
        """Handle hardware feedback via LogicManager"""
        if self.logic.process_feedback(msg.data):
            self.get_logger().info("Hardware action confirmed complete.")

    def tf_callback(self, msg):
        """Update robot position from TF"""
        for transform in msg.transforms:
            if transform.child_frame_id == 'khr2026_robot':
                self.state.robot_x = transform.transform.translation.x
                self.state.robot_y = transform.transform.translation.y

    def score_callback(self, msg):
        """Update internal GameState via LogicManager"""
        self.logic.sync_state(msg.data)

    def control_loop(self):
        # 1. Check if moving or executing hardware
        if self.current_action:
            if not self.navigator.isTaskComplete():
                return # Still navigating to spot
            
            # Check for Navigation result
            nav_result = self.navigator.getResult()
            if nav_result == TaskResult.FAILED:
                self.get_logger().error(f"Navigation to {self.current_action['name']} FAILED. Replanning...")
                self.current_action = None
                return
            elif nav_result == TaskResult.CANCELED:
                self.get_logger().warn(f"Navigation to {self.current_action['name']} CANCELED.")
                self.current_action = None
                return

            # Arrived! Execute hardware action if not started
            if not self.logic.is_executing_hardware:
                if self.last_hardware_cmd_time == 0:
                    self.get_logger().info(f"Arrived at {self.current_action['name']}. Starting hardware sequence...")
                    self.execute_hardware_sequence(self.current_action)
                    self.logic.prepare_action(
                        self.current_action['type'], 
                        gain_y=self.current_action.get('gain_y', 0),
                        gain_r=self.current_action.get('gain_r', 0),
                        mech_index=1 
                    )
                    self.last_hardware_cmd_time = time.time()
                return

            # Timeout (15s)
            if time.time() - self.last_hardware_cmd_time > 15.0:
                self.get_logger().warn("Hardware timeout. Skipping action...")
                self.logic.is_executing_hardware = False
                self.logic.waiting_for_feedback_id = None

            if self.logic.is_executing_hardware:
                return # Still waiting for module to finish

            # Action complete
            self.get_logger().info(f"Mission step at {self.current_action['name']} complete.")
            self.update_state_post_action(self.current_action)
            self.current_action = None
            self.last_hardware_cmd_time = 0
            
        # 2. Plan
        plan = self.planner.plan_mission(self.state)
        if not plan: return
            
        next_spot = plan[0]
        self.get_logger().info(f"Navigating to: {next_spot['name']}")
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = next_spot['x']
        goal.pose.position.y = next_spot['y']
        goal.pose.orientation.w = 1.0
        
        self.navigator.goToPose(goal)
        self.current_action = next_spot

    def execute_hardware_sequence(self, action):
        """Send JSON commands to /robot_control based on spot type"""
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
                control_data["ring"]["1_state"] = "open"
        
        elif action['type'] == 'target':
            control_data["yagura"]["1_pos"] = "down"
            control_data["yagura"]["1_state"] = "open"
            
        elif action['type'] == 'honmaru':
            control_data["ring"]["1_pos"] = "honmaru"
            control_data["ring"]["1_state"] = "closed"
            
        self.control_pub.publish(String(data=json.dumps(control_data)))

    def update_state_post_action(self, action):
        """Update internal state prediction"""
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
