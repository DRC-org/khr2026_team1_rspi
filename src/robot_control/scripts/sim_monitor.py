#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import json
import math

class SimMonitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        self.create_subscription(String, '/score_detail', self.score_cb, 10)
        self.create_subscription(String, '/robot_control', self.control_cb, 10)
        # Publisher for arrival notification (used by MissionControlNode)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.pos = (0.0, 0.0)
        self.score = 0
        self.ote = False
        self.v_goal = False
        self.last_cmd = "{}"
        self.create_timer(1.0, self.render)
        print("\033[2J\033[H")  # Clear screen

    # ---------------------------------------------------------------------
    def tf_cb(self, msg):
        for t in msg.transforms:
            # Gazebo bridge publishes odom→base_link, not khr2026_robot
            if t.child_frame_id == 'base_link':
                self.pos = (t.transform.translation.x, t.transform.translation.y)

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

        # 簡易マップ表示 (3.5m x 7.0m)
        grid_w, grid_h = 20, 14
        print("\n[フィールド簡易マップ]")
        for y in range(grid_h, -1, -1):
            line = ""
            for x in range(grid_w):
                rx, ry = self.pos[0] * (grid_w/3.5), self.pos[1] * (grid_h/7.0)
                if abs(x - rx) < 0.5 and abs(y - ry) < 0.5:
                    line += "🤖"
                elif (x == int(3.25*(grid_w/3.5)) and y == int(1.4*(grid_h/7.0))):
                    line += "👑"
                elif x == 0 or x == grid_w-1 or y == 0 or y == grid_h-1:
                    line += "田"
                else:
                    line += "  "
            print(line)

def main():
    rclpy.init()
    node = SimMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
