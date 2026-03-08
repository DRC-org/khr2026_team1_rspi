import json
import math
import os
import threading
import time

import rclpy.duration
import rclpy.time
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


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
        self._pending_goal_xyz: tuple[float, float, float] | None = None
        self._goal_retry_count: int = 0
        self._nav_failure_retry_count: int = 0

        pkg_dir = get_package_share_directory("auto_nav")
        wp_path = os.path.join(pkg_dir, "config", "waypoints.yaml")
        with open(wp_path) as f:
            data = yaml.safe_load(f)
            self._waypoints = data.get("waypoints", {})
            self._auto_seq_names: list[str] = data.get("auto_sequence", [])

        # field_dimensions.yaml からコート別初期位置を読み込む
        dims_path = os.path.join(pkg_dir, "config", "field_dimensions.yaml")
        with open(dims_path) as f:
            dims = yaml.safe_load(f)
        self._start_positions: dict = dims.get("start_positions", {})

        self._sub = self.create_subscription(String, "bluetooth_rx", self._on_bt_rx, 10)
        self._pub_mode = self.create_publisher(String, "/nav_mode", 10)
        self._pub_tx = self.create_publisher(String, "bluetooth_tx", 10)
        # on_arrive の hand_control コマンドを robot_control に転送するために使用
        self._pub_rx = self.create_publisher(String, "bluetooth_rx", 10)
        # 手動移動後に slam_toolbox のスキャンマッチング起点を更新するために使用
        self._pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self._robot_pos = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self._sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._timer_pos = self.create_timer(0.2, self._publish_robot_pos)

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_markers = self.create_publisher(MarkerArray, "/waypoint_markers", 10)
        self.create_timer(1.0, self._publish_waypoint_markers)

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
            from_index = int(data.get("from_index", 0))
            self._handle_start_auto(from_index)
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

        nav_msg: dict = {"nav_status": "navigating", "waypoint": name}
        if self._auto_seq_running:
            nav_msg["seq_index"] = self._auto_seq_index
            nav_msg["seq_total"] = len(self._auto_seq_names)
        self._pub_tx.publish(String(data=json.dumps(nav_msg)))

    def _send_goal(self, x: float, y: float, theta: float) -> None:
        self._pending_goal_xyz = (x, y, theta)
        self._goal_retry_count = 0
        self._nav_failure_retry_count = 0
        self._send_goal_impl(x, y, theta)

    def _send_goal_impl(self, x: float, y: float, theta: float) -> None:
        # navigate_to_pose サーバー未起動の場合は 5 秒待ってタイムアウト
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose server not available")
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "navigate_to_pose server not available",
                "seq_index": self._auto_seq_index,
                "seq_total": len(self._auto_seq_names),
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
            # bt_navigator が active 状態になる前にゴールが届いた場合はリトライ
            _MAX_RETRIES = 3
            if self._state == "NAVIGATING" and self._goal_retry_count < _MAX_RETRIES:
                self._goal_retry_count += 1
                self.get_logger().warn(
                    f"Goal rejected (bt_navigator not yet active?), "
                    f"retry {self._goal_retry_count}/{_MAX_RETRIES} in 2s..."
                )
                t = threading.Timer(2.0, self._retry_send_goal)
                t.daemon = True
                t.start()
                return
            self.get_logger().warn("Goal rejected by NavigateToPose server (all retries exhausted)")
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": "navigate_to_pose server rejected goal",
                "seq_index": self._auto_seq_index,
                "seq_total": len(self._auto_seq_names),
            })))
            return
        self._goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _retry_send_goal(self) -> None:
        if self._pending_goal_xyz and self._state == "NAVIGATING":
            x, y, theta = self._pending_goal_xyz
            self._send_goal_impl(x, y, theta)

    def _result_cb(self, future) -> None:
        from action_msgs.msg import GoalStatus

        result = future.result()

        # キャンセル済みの場合は stop_auto/_handle_mode 側で処理済み
        if result.status == GoalStatus.STATUS_CANCELED:
            self._state = "AUTO_IDLE"
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            _NAV_FAILURE_MAX_RETRIES = 5
            if (
                self._auto_seq_running
                and self._state == "NAVIGATING"
                and self._nav_failure_retry_count < _NAV_FAILURE_MAX_RETRIES
            ):
                self._nav_failure_retry_count += 1
                self.get_logger().warn(
                    f"Navigation failed (status={result.status}), "
                    f"retrying goal in 3s... "
                    f"({self._nav_failure_retry_count}/{_NAV_FAILURE_MAX_RETRIES})"
                )
                t = threading.Timer(3.0, self._retry_send_goal)
                t.daemon = True
                t.start()
                return

            # リトライ上限到達またはシーケンス外の失敗
            self._state = "AUTO_IDLE"
            self._auto_seq_running = False
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "waypoint": self._current_goal_name,
                "message": f"navigation failed with status {result.status}",
                "seq_index": self._auto_seq_index,
                "seq_total": len(self._auto_seq_names),
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
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "arrived",
                "waypoint": self._current_goal_name,
                "seq_index": self._auto_seq_index,
                "seq_total": len(self._auto_seq_names),
            })))
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

        # AMCL にロボット初期位置を通知（field_dimensions.yaml の start_positions を使用）
        self._publish_start_pose(court)

        self._pub_tx.publish(String(data=json.dumps({
            "nav_status": "court_set",
            "court": court,
        })))

    def _publish_start_pose(self, court: str) -> None:
        pos = self._start_positions.get(court)
        if not pos:
            self.get_logger().warn(f"start_positions に {court} が定義されていません")
            return
        x: float = pos["x"]
        y: float = pos["y"]
        yaw: float = pos["yaw"]

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # 0.5s 前のスタンプを使用: EKF が確実に発行済みの TF が存在し
        # "extrapolation into the future" エラーを防ぐ
        msg.header.stamp = (
            self.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
        ).to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 初期位置の不確かさ: ±0.5m・±0.2 rad
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.04  # yaw
        self._pub_initial_pose.publish(msg)

    def _handle_start_auto(self, from_index: int = 0) -> None:
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
        if from_index < 0 or from_index >= len(self._auto_seq_names):
            self._pub_tx.publish(String(data=json.dumps({
                "nav_status": "error",
                "message": f"invalid from_index: {from_index}",
            })))
            return

        self._auto_seq_running = True
        self._auto_seq_index = from_index
        self._sequence_abort.clear()

        # from_index=0 でも initialpose を publish して AMCL 収束を待ってから開始
        t = threading.Thread(
            target=self._relocate_and_start,
            args=(from_index,),
            daemon=True,
        )
        t.start()

    def _relocate_and_start(self, from_index: int) -> None:
        if from_index == 0:
            # シーケンス開始: コート別スタート位置で AMCL を初期化
            self._publish_start_pose(self._current_court)
        else:
            # 手動移動後のリローカライズ: 移動先ウェイポイント位置で初期化
            wp_name = self._auto_seq_names[from_index]
            self._publish_initial_pose_at_waypoint(wp_name)

        wp_name = self._auto_seq_names[from_index]

        # map→odom TF が確立されるまで待機（AMCL ローカライズ完了 + Nav2 コストマップ更新を保証）
        # 固定秒数ではなく実際の TF 存在チェックで判定するため、マップ読み込みの遅延に対応できる
        _WAIT_TIMEOUT = 30.0
        _PUBLISH_INTERVAL = 1.0
        deadline = time.monotonic() + _WAIT_TIMEOUT
        last_pub = 0.0

        self.get_logger().info("Waiting for AMCL to establish map→odom TF...")
        tf_ready = False
        while time.monotonic() < deadline:
            if self._sequence_abort.is_set():
                return
            try:
                self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                tf_ready = True
                break
            except Exception:
                pass

            now = time.monotonic()
            if now - last_pub >= _PUBLISH_INTERVAL:
                last_pub = now
                remaining = max(0, int(deadline - now))
                self._pub_tx.publish(String(data=json.dumps({
                    "nav_status": "relocating",
                    "countdown": remaining,
                    "waypoint": wp_name,
                    "seq_index": from_index,
                    "seq_total": len(self._auto_seq_names),
                })))
            time.sleep(0.2)

        if not tf_ready:
            self.get_logger().warn("map→odom TF not available after timeout, proceeding anyway")
        else:
            self.get_logger().info("map→odom TF confirmed, starting navigation")

        if self._sequence_abort.is_set():
            return
        self._advance_auto_sequence()

    def _publish_initial_pose_at_waypoint(self, wp_name: str) -> None:
        wp = self._waypoints.get(wp_name)
        if not wp:
            return
        x, y, theta = self._apply_court_transform(wp["x"], wp["y"], wp.get("theta", 0.0))

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # 0.5s 前のスタンプを使用: EKF が確実に発行済みの TF が存在し
        # "extrapolation into the future" エラーを防ぐ
        msg.header.stamp = (
            self.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
        ).to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # 位置 ±0.5m・角度 ±0.2 rad 程度を想定した共分散
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.04  # yaw
        self._pub_initial_pose.publish(msg)

    def _handle_stop_auto(self) -> None:
        self._cancel_goal()
        self._sequence_abort.set()
        self._auto_seq_running = False
        self._state = "AUTO_IDLE"

    def _advance_auto_sequence(self) -> None:
        name = self._auto_seq_names[self._auto_seq_index]
        self._handle_goal({"type": "nav_goal", "waypoint": name})

    def _publish_waypoint_markers(self) -> None:
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, name in enumerate(self._auto_seq_names):
            wp = self._waypoints.get(name)
            if not wp:
                continue
            x, y, _ = self._apply_court_transform(wp["x"], wp["y"], wp.get("theta", 0.0))

            is_current = (
                name == self._current_goal_name
                and self._state in ("NAVIGATING", "SEQUENCE")
            )
            is_visited = self._auto_seq_running and i < self._auto_seq_index

            # 球マーカー
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            if is_current:
                m.scale.x = m.scale.y = m.scale.z = 0.30
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            elif is_visited:
                m.scale.x = m.scale.y = m.scale.z = 0.15
                m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.6
            else:
                m.scale.x = m.scale.y = m.scale.z = 0.20
                m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.7, 1.0, 0.9
            markers.markers.append(m)

            # テキストラベル
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = now
            t.ns = "labels"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = name
            markers.markers.append(t)

        # 残りルート（LINE_STRIP）
        if self._auto_seq_running:
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = now
            line.ns = "route"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.04
            line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.3, 0.8
            for name in self._auto_seq_names[self._auto_seq_index:]:
                wp = self._waypoints.get(name)
                if not wp:
                    continue
                x, y, _ = self._apply_court_transform(wp["x"], wp["y"], wp.get("theta", 0.0))
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.05
                line.points.append(p)
            markers.markers.append(line)
        else:
            del_marker = Marker()
            del_marker.ns = "route"
            del_marker.id = 0
            del_marker.action = Marker.DELETE
            markers.markers.append(del_marker)

        self._pub_markers.publish(markers)

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
