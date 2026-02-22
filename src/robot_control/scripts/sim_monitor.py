#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
from geometry_msgs.msg import TransformStamped
import json
import math
import time

class SimMonitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        
        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(String, '/score_detail', self.score_cb, 10)
        self.create_subscription(String, '/robot_control', self.control_cb, 10)
        
        # Publisher for arrival notification
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.pos = (0.0, 0.0)
        self.score = 0
        self.ote = False
        self.v_goal = False
        self.last_cmd = "{}"
        self.create_timer(1.0, self.render)
        self.create_timer(0.1, self.update_pos) # Faster position update
        print("\033[2J\033[H")  # Clear screen

    def update_pos(self):
        """Update position from TFBuffer (Map -> BaseLink)"""
        try:
            # map -> base_link を取得
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            self.pos = (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    # ---------------------------------------------------------------------
    def tf_cb(self, msg):
        # Deprecated: using Buffer instead
        pass

    # ---------------------------------------------------------------------
    def score_cb(self, msg):
        data = json.loads(msg.data)
        self.score = data.get('total_score', 0)
        self.ote = data.get('ote', False)
        self.v_goal = data.get('v_goal', False)

    # ---------------------------------------------------------------------
    def control_cb(self, msg):
        self.last_cmd = msg.data

    # ---------------------------------------------------------------------
    def _extract_target(self):
        """Extract target coordinates from the latest /robot_control JSON.
        Expected format: {"target": {"x": <float>, "y": <float>}}
        Returns (x, y) or None if parsing fails.
        """
        try:
            data = json.loads(self.last_cmd)
            tgt = data.get('target')
            if tgt and 'x' in tgt and 'y' in tgt:
                return float(tgt['x']), float(tgt['y'])
        except Exception:
            pass
        return None

    # ---------------------------------------------------------------------
    def render(self):
        # Check arrival condition (within 0.2 m of target)
        target = self._extract_target()
        if target:
            dx = self.pos[0] - target[0]
            dy = self.pos[1] - target[1]
            if math.hypot(dx, dy) < 0.2:
                self.status_pub.publish(String(data="arrived"))

        # ---- UI ----
        print("\033[H")  # Home
        print("=== KHR 2026 自律制御シミュレーション実況 ===")
        print(f"ロボット位置: X={self.pos[0]:.2f}, Y={self.pos[1]:.2f}")
        print(f"現在の得点  : {self.score} 点")
        print(f"状態判定    : {'[王手!]' if self.ote else ''} {'[攻略達成(V-GOAL)!]' if self.v_goal else '進行中'}")
        print("-" * 40)
        print(f"最新の指令  : {self.last_cmd}")
        print("-" * 40)

def main():
    rclpy.init()
    node = SimMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
