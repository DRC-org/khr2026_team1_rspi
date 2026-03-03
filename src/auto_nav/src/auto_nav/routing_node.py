import json
import math
import os
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class RoutingNode(Node):
    """
    Bluetooth コマンドを受け取り Nav2 の NavigateToPose ActionServer にゴールを送信するノード。

    状態遷移:
      MANUAL ──nav_mode:auto──→ AUTO_IDLE ──nav_goal/start_auto──→ NAVIGATING
        ↑                          ↑   ↑                                │
        └──nav_mode:manual──────────┘   └── AUTO_IDLE ←── SEQUENCE ←───┘
                                             ↑
                                    全ウェイポイント完了
    """

    def __init__(self):
        super().__init__("routing_node")
        self._state = "MANUAL"
        self._current_goal_name = None
        self._goal_handle = None
        self._current_court: str = "blue"

        pkg_dir = get_package_share_directory("auto_nav")
        wp_path = os.path.join(pkg_dir, "config", "waypoints.yaml")
        with open(wp_path) as f:
            data = yaml.safe_load(f)
            self._waypoints = data.get("waypoints", {})
            self._auto_seq_names: list[str] = data.get("auto_sequence", [])

        self._sub = self.create_subscription(String, "bluetooth_rx", self._on_bt_rx, 10)
        self._pub_mode = self.create_publisher(String, "/nav_mode", 10)
        self._pub_tx = self.create_publisher(String, "bluetooth_tx", 10)
        # on_arrive の hand_control コマンドを robot_control に転送するために使用
        self._pub_rx = self.create_publisher(String, "bluetooth_rx", 10)

        self._robot_pos = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self._sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._timer_pos = self.create_timer(0.2, self._publish_robot_pos)

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._sequence_abort = threading.Event()
        self._sequence_thread: threading.Thread | None = None
        self._auto_seq_running: bool = False
        self._auto_seq_index: int = 0

    def _on_odom(self, msg: Odometry) -> None:
        x_mm = msg.pose.pose.position.x * 1000
        y_mm = msg.pose.pose.position.y * 1000
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        angle_deg = math.degrees(2 * math.atan2(qz, qw))
        self._robot_pos = {"x": x_mm, "y": y_mm, "angle": angle_deg}

    def _publish_robot_pos(self) -> None:
        pos = self._robot_pos
        self._pub_tx.publish(String(data=json.dumps({
            "type": "robot_pos",
            "x": pos["x"],
            "y": pos["y"],
            "angle": pos["angle"],
        })))

    def _on_bt_rx(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type")
        if msg_type == "nav_mode":
            self._handle_mode(data.get("mode", ""))
        elif msg_type == "nav_goal":
            self._handle_goal(data)
        elif msg_type == "set_court":
            self._handle_set_court(data)
        elif msg_type == "start_auto":
            self._handle_start_auto()
        elif msg_type == "stop_auto":
            self._handle_stop_auto()

    def _handle_mode(self, mode: str) -> None:
        if mode not in ("manual", "auto"):
            return
        if mode == "manual":
            if self._state == "NAVIGATING":
                self._cancel_goal()
            self._sequence_abort.set()
            self._auto_seq_running = False
        self._state = "MANUAL" if mode == "manual" else "AUTO_IDLE"
        self._pub_mode.publish(String(data=mode))
        self._pub_tx.publish(String(data=json.dumps({"nav_status": "mode", "mode": mode})))

    def _handle_goal(self, data: dict) -> None:
        if self._state == "MANUAL":
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "nav_goal requires auto mode",
            })))
            return

        if "waypoint" in data:
            name = data["waypoint"]
            if name not in self._waypoints:
                self._pub_tx.publish(String(data=json.dumps({
                    "nav_status": "error",
                    "message": f"unknown waypoint: {name}",
                })))
                return
            wp = self._waypoints[name]
            x, y, theta = wp["x"], wp["y"], wp["theta"]
        else:
            name = "direct"
            x, y, theta = data["x"], data["y"], data.get("theta", 0.0)

        x, y, theta = self._apply_court_transform(x, y, theta)
        self._current_goal_name = name
        self._state = "NAVIGATING"
        self._send_goal(x, y, theta)
        self._pub_tx.publish(String(data=json.dumps({
            "nav_status": "navigating",
            "waypoint": name,
        })))

    def _send_goal(self, x: float, y: float, theta: float) -> None:
        # navigate_to_pose サーバー未起動の場合は 5 秒待ってタイムアウト
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose server not available")
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "navigate_to_pose server not available",
            })))
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # yaw → quaternion（z 軸周りのみ）
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Goal rejected by NavigateToPose server")
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            return
        self._goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        from action_msgs.msg import GoalStatus

        result = future.result()

        # キャンセル済みの場合は stop_auto/_handle_mode 側で処理済み
        if result.status == GoalStatus.STATUS_CANCELED:
            self._state = "AUTO_IDLE"
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "waypoint": self._current_goal_name,
                "message": f"navigation failed with status {result.status}",
            })))
            return

        wp = self._waypoints.get(self._current_goal_name, {})
        on_arrive = wp.get("on_arrive", [])

        if on_arrive:
            self._state = "SEQUENCE"
            self._sequence_abort.clear()
            t = threading.Thread(
                target=self._run_on_arrive_sequence,
                args=(on_arrive, self._on_sequence_done),
                daemon=True,
            )
            self._sequence_thread = t
            t.start()
        else:
            self._on_sequence_done()

    def _run_on_arrive_sequence(self, actions: list, done_callback) -> None:
        for act in actions:
            if self._sequence_abort.is_set():
                return
            action_type = act.get("action")
            if action_type == "hand_control":
                cmd = {
                    "type": "hand_control",
                    "target": act.get("target", ""),
                    "control_type": act.get("control_type", ""),
                    "action": act.get("action_value", ""),
                }
                self._pub_rx.publish(String(data=json.dumps(cmd)))
            elif action_type == "wait":
                duration = act.get("duration", 0.0)
                elapsed = 0.0
                interval = 0.05
                while elapsed < duration:
                    if self._sequence_abort.is_set():
                        return
                    time.sleep(interval)
                    elapsed += interval
        done_callback()

    def _on_sequence_done(self) -> None:
        if self._auto_seq_running:
            self._auto_seq_index += 1
            if self._auto_seq_index < len(self._auto_seq_names):
                self._advance_auto_sequence()
                return
            else:
                self._auto_seq_running = False
                self._state = "AUTO_IDLE"
                self._pub_tx.publish(String(data=json.dumps({
                    "nav_status": "completed",
                })))
                return

        self._state = "AUTO_IDLE"
        self._pub_tx.publish(String(data=json.dumps({
            "nav_status": "arrived",
            "waypoint": self._current_goal_name,
        })))

    def _apply_court_transform(
        self, x: float, y: float, theta: float
    ) -> tuple[float, float, float]:
        # 赤コートは青コートに対して北南軸（Y軸）対称: X反転
        if self._current_court == "red":
            return -x, y, math.pi - theta
        return x, y, theta

    def _handle_set_court(self, data: dict) -> None:
        court = data.get("court", "blue")
        if court not in ("blue", "red"):
            return
        self._current_court = court
        self._pub_tx.publish(String(data=json.dumps({
            "nav_status": "court_set",
            "court": court,
        })))

    def _handle_start_auto(self) -> None:
        if self._state == "MANUAL":
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "start_auto requires auto mode",
            })))
            return
        if not self._auto_seq_names:
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "auto_sequence is empty",
            })))
            return
        self._auto_seq_running = True
        self._auto_seq_index = 0
        self._sequence_abort.clear()
        self._advance_auto_sequence()

    def _handle_stop_auto(self) -> None:
        self._cancel_goal()
        self._sequence_abort.set()
        self._auto_seq_running = False
        self._state = "AUTO_IDLE"

    def _advance_auto_sequence(self) -> None:
        name = self._auto_seq_names[self._auto_seq_index]
        self._handle_goal({"type": "nav_goal", "waypoint": name})

    def _cancel_goal(self) -> None:
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._pub_tx.publish(String(data=json.dumps({"nav_status": "cancelled"})))


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = RoutingNode()
    # MultiThreadedExecutor を使用: ActionClient の Waitable が SingleThreadedExecutor の
    # Subscription コールバックをブロックする問題を回避するため
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
