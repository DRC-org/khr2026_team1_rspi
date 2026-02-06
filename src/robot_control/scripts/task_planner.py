#!/usr/bin/env python3
import sys
import math
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# OR-Tools check
try:
    from ortools.sat.python import cp_model
except ImportError:
    print("Error: ortools not installed. Please run: pip install ortools")
    sys.exit(1)

# Added import
from task_optimizer_standalone import TaskOptimizer


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')
        
        # Publisher for path visualization
        self.path_publisher = self.create_publisher(String, '/task_planner/planned_path', 10)
        
        self.optimizer = TaskOptimizer()
        self.navigator = BasicNavigator()
        
        self.get_logger().info("Task Planner Node initialized")
    
    def publish_path(self, path_ids):
        """Publish planned path for visualization"""
        path_data = {
            "spots": self.optimizer.spots,
            "path": path_ids,
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(path_data)
        self.path_publisher.publish(msg)
        self.get_logger().info(f"Published path: {[self.optimizer.spots[pid]['name'] for pid in path_ids]}")
    
    def execute_mission(self):
        """Main mission execution logic"""
        # Set Initial Pose
        start_spot = self.optimizer.spots[0]
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = start_spot["x"]
        initial_pose.pose.position.y = start_spot["y"]
        initial_pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Setting initial pose to {start_spot['name']}")
        self.navigator.setInitialPose(initial_pose)
        
        time.sleep(2.0)
        
        # Solve for path
        path_ids = self.optimizer.solve(start_node=0, time_limit=180)
        self.get_logger().info(f"Planned execution order: {[self.optimizer.spots[pid]['name'] for pid in path_ids]}")
        
        # Publish path for visualization
        self.publish_path(path_ids)
        
        if not path_ids:
            self.get_logger().warn("Empty path. Cannot execute mission.")
            return
        
        # Execution Loop
        if path_ids and path_ids[0] == 0:
            path_ids.pop(0)
        
        for spot_id in path_ids:
            spot = self.optimizer.spots[spot_id]
            self.get_logger().info(f"Navigating to target: {spot['name']} ({spot['x']}, {spot['y']})")
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = spot['x']
            goal_pose.pose.position.y = spot['y']
            goal_pose.pose.orientation.w = 1.0
            
            self.navigator.goToPose(goal_pose)
            
            while not self.navigator.isTaskComplete():
                pass
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Reached {spot['name']}. Executing action...")
                time.sleep(2.0)
                self.get_logger().info("Action complete.")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"Task canceled for {spot['name']}")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"Task failed for {spot['name']}")
                break
        
        self.get_logger().info("Mission Complete.")


def main():
    rclpy.init()
    
    node = TaskPlannerNode()
    
    try:
        node.execute_mission()
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
