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

class TaskOptimizer:
    def __init__(self):
        # Coordinates: Based on Field 3.5m x 7.0m, Origin (0,0) at bottom-left approx.
        # Adjusted based on SVG analysis
        self.spots = {
            0: {"name": "Start", "x": 0.5, "y": 0.5, "score": 0, "required_for_vgoal": False},
            
            # 陣取り (Jintori) - Side areas
            1: {"name": "Jintori_1", "x": 0.5, "y": 2.0, "score": 10, "required_for_vgoal": True},
            2: {"name": "Jintori_2", "x": 0.5, "y": 3.5, "score": 10, "required_for_vgoal": True},
            3: {"name": "Jintori_3", "x": 0.5, "y": 5.0, "score": 10, "required_for_vgoal": True},
            
            # 中央 (Center) - High value
            4: {"name": "Center_Area", "x": 1.75, "y": 3.5, "score": 30, "required_for_vgoal": False},
            
            # 本丸 (Honmaru) - Critical for V-Goal
            5: {"name": "Honmaru", "x": 1.75, "y": 6.0, "score": 50, "required_for_vgoal": True},
            
            # リング (Ring) - Additional points
            6: {"name": "Ring_Zone", "x": 2.5, "y": 2.5, "score": 20, "required_for_vgoal": False},
        }
        
        # Robot / Task parameters
        self.params = {
            "robot_speed": 0.4,       # m/s (Conservative estimate)
            "time_limit": 180,        # seconds (Match duration)
            "action_time_per_spot": 10.0, # seconds (Stop, perform action, verify)
            "v_goal_bonus": 10000,    # Huge bonus to prioritize V-Goal
        }

    def calculate_distance(self, id1, id2):
        s1 = self.spots[id1]
        s2 = self.spots[id2]
        return math.sqrt((s1["x"] - s2["x"])**2 + (s1["y"] - s2["y"])**2)

    def solve(self, start_node_id=0, remaining_time=180):
        print(f"[Optimizer] Solving path from {self.spots[start_node_id]['name']} with {remaining_time}s left...")
        
        model = cp_model.CpModel()
        all_spots = list(self.spots.keys())
        
        # Variables
        # x[i, j]: transition from i to j
        x = {}
        for i in all_spots:
            for j in all_spots:
                x[i, j] = model.NewBoolVar(f'x_{i}_{j}')

        # visit[i]: checks if spot i is visited
        visit = {i: model.NewBoolVar(f'visit_{i}') for i in all_spots}
        
        # Constraints
        # 1. Start node is visited
        model.Add(visit[start_node_id] == 1)
        
        # 2. Flow constraints (Circuit) - Simplified TSP/Orienteering
        # To handle partial visits (skipping nodes), we treat skipped nodes as self-loops x[i,i]=1 
        # OR connect them to a dummy node. 
        # Standard approach for Orienteering with Time Window in CP-SAT:
        # Use simple flow conservation but Circuit is robust.
        # Let's use circuit with self-loops allowed for skipped nodes.
        
        circuit_arcs = []
        for i in all_spots:
            for j in all_spots:
                circuit_arcs.append([i, j, x[i, j]])
                
        model.AddCircuit(circuit_arcs)
        
        # Link visit[i] to self-loops
        # If x[i, i] is TRUE, then visit[i] is FALSE (unless it's the start node?? No, usually skip means skip)
        # But we need to "visit" the start node even if we loop there? 
        # Let's adjust: Start node CANNOT self-loop if we act. 
        # Actually easier: visit[i] == Not(x[i,i])
        for i in all_spots:
            if i == start_node_id:
                # Start node implies visited logic handled separately or force exit?
                # If we produce a path, we visit start.
                pass 
            else:
                model.Add(visit[i] == x[i, i].Not())

        # 3. Time Budget
        # Scale time to integer
        SCALE = 10 
        max_time_scaled = int(remaining_time * SCALE)
        path_time = model.NewIntVar(0, max_time_scaled * 2, 'path_time') # Allow bounds
        
        travel_times = []
        for i in all_spots:
            for j in all_spots:
                if i == j:
                    # Self loop = 0 time
                    travel_times.append(x[i, j] * 0)
                else:
                    dist = self.calculate_distance(i, j)
                    time_s = (dist / self.params["robot_speed"]) + self.params["action_time_per_spot"]
                    time_val = int(time_s * SCALE)
                    travel_times.append(x[i, j] * time_val)
        
        model.Add(path_time == sum(travel_times))
        model.Add(path_time <= max_time_scaled)
        
        # 4. Objective: V-Goal > Score
        v_goal_nodes = [i for i, s in self.spots.items() if s["required_for_vgoal"]]
        v_goal_achieved = model.NewBoolVar('v_goal_achieved')
        
        # v_goal_achieved <=> Sum(visited required) == Total required
        required_count = len(v_goal_nodes)
        visited_required = sum(visit[k] for k in v_goal_nodes)
        
        model.Add(visited_required == required_count).OnlyEnforceIf(v_goal_achieved)
        model.Add(visited_required < required_count).OnlyEnforceIf(v_goal_achieved.Not())
        
        # Calculate raw score
        total_score = sum(visit[i] * int(spot["score"]) for i, spot in self.spots.items())
        
        # Objective function
        model.Maximize(total_score + (v_goal_achieved * self.params["v_goal_bonus"]))
        
        # Solve
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 2.0 # Fast solve
        status = solver.Solve(model)
        
        if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
            print(f"[Optimizer] Solution Found: {solver.StatusName(status)}")
            print(f"  V-Goal Possible: {bool(solver.Value(v_goal_achieved))}")
            print(f"  Projected Score: {solver.Value(total_score)}")
            
            # Reconstruct Path
            path = []
            curr = start_node_id
            visited_set = set()
            
            while True:
                path.append(curr)
                visited_set.add(curr)
                
                next_node = -1
                for j in all_spots:
                    if solver.Value(x[curr, j]):
                        next_node = j
                        break
                
                if next_node == -1:
                    break
                if next_node == curr: # Self-loop means end of actual path logic in circuit with skips
                    # Check if there are other visited nodes "after" this? 
                    # In Circuit with self-loops for skips, the active path is a single cycle.
                    # Since we start at start_node_id, the cycle includes start_node.
                    # The parts x[k,k]=1 are disconnected components of size 1.
                    break
                if next_node in visited_set:
                    # Cycle closed (return to start)
                    # If start is not the end goal (usually start != end in robot tasks unless specified),
                    # we should handle "stay at end". 
                    # For now, append return to start if that's what solver did.
                    path.append(next_node) 
                    break
                
                curr = next_node
                
            return path
        else:
            print("[Optimizer] No solution found.")
            return []


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
        path_ids = self.optimizer.solve(start_node_id=0, remaining_time=180)
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
