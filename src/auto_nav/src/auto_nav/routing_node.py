import json
import math
import os
import threading
import time

import numpy as np
import rclpy.duration
import rclpy.time
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_msgs.msg import HandMessage, WheelMessage
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# MPPI time_steps（固定）
MPPI_TIME_STEPS = 8

# 直接アプローチ用定数
DIRECT_APPROACH_DIST = 0.3      # 0.3m 以内で直接アプローチに切り替え
GOAL_REACHED_DIST = 0.02        # ゴール到達判定距離 [m]
DIRECT_APPROACH_SPEED = 0.15    # 直接アプローチ時の移動速度 [m/s]
DIRECT_APPROACH_TIMEOUT = 5.0   # 直接アプローチのタイムアウト [s]

# 運動学定数（cmd_vel_bridge_node.py と同じ値）
WHEEL_RADIUS = 0.04925
GEAR_RATIO = 19.20320855614973
L_X = 0.1725
L_Y = 0.2425
G = L_X + L_Y
MAX_RPM = 12000.0
MIN_RPM_FWD = 600.0
MIN_RPM_LAT = 800.0

# 並進 + 回転 P 制御
KP_POS = 2.0          # [RPM / (m/s)] 的な無次元ゲイン（実効速度は MIN_RPM で底上げ）
KP_YAW = 4.0          # [rad/s / rad]
MAX_OMEGA = 1.5       # 最大角速度 [rad/s]
YAW_TOLERANCE = 0.04  # ゴール到達判定の角度許容誤差 [rad] ≈ 2.3°

# LiDAR 前方補完停止
LIDAR_FORWARD_ANGLE = 0.35  # 前方スキャン半角 [rad] ≈ 20°
LIDAR_STOP_DIST = 0.05      # 前方障害物検知で停止する距離 [m]
MS_TO_RPM = (60.0 * GEAR_RATIO) / (2.0 * math.pi * WHEEL_RADIUS)

# 壁際後退: 壁密着 waypoint から次へ向かう前にロボットを後退させる
# inscribed_radius(0.25m) を超える距離が必要
WALL_BACK_OFF_DIST = 0.35       # 後退距離 [m]
BACK_OFF_SPEED = 0.5            # 後退速度 [m/s]
BACK_OFF_TIMEOUT = 5.0          # 後退タイムアウト [s]
BACK_OFF_REACHED_DIST = 0.05    # 後退到達判定距離 [m]

# 櫓補正用定数
YAGURA_CORRECTION_DIST = 3.0       # 補正を試みる残距離 [m]
YAGURA_CORRECTION_MAX = 0.3        # 補正量の上限 [m]

# ring_pickup_1 用 MPPI プリセット
MPPI_RING_PICKUP_1 = {
    "FollowPath.PathFollowCritic.cost_weight": 12.0,
    "FollowPath.PathAlignCritic.cost_weight": 18.0,
    "FollowPath.CostCritic.cost_weight": 1.8,
    "FollowPath.CostCritic.consider_footprint": True,
}
MPPI_DEFAULT = {
    "FollowPath.PathFollowCritic.cost_weight": 5.0,
    "FollowPath.PathAlignCritic.cost_weight": 10.0,
    "FollowPath.CostCritic.cost_weight": 2.5,
    "FollowPath.CostCritic.consider_footprint": False,
}

_YAGURA_POS_MAP = {"up": 0, "down": 1, "stopped": 2, "up_done": 3, "down_done": 4}
_YAGURA_STATE_MAP = {"open": 0, "close": 1, "stopped": 2, "open_done": 3, "close_done": 4}
_RING_POS_MAP = {"pickup": 0, "yagura": 1, "honmaru": 2, "stopped": 3, "pickup_done": 4, "yagura_done": 5, "honmaru_done": 6}
_RING_STATE_MAP = {"open": 0, "close": 1, "stopped": 2, "open_done": 3, "close_done": 4, "grip_fail": 5}


class RoutingNode(Node):
    """
    Bluetooth コマンドを受け取り Nav2 の NavigateToPose ActionServer にゴールを送信するノード。

    状態遷移:
      MANUAL ──nav_mode:auto──→ AUTO_IDLE ──nav_goal/start_auto──→ NAVIGATING
        ↑                          ↑   ↑                                │
        └──nav_mode:manual──────────┘   └── AUTO_IDLE ←── SEQUENCE ←───┘
                                             ↑               ↑
                                    全ウェイポイント完了       │
                                                     DIRECT_APPROACH
                                                (距離<0.5m で Nav2 キャンセル→直進)
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
        self._timer_pos = self.create_timer(0.5, self._publish_robot_pos)

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_markers = self.create_publisher(MarkerArray, "/waypoint_markers", 10)
        self.create_timer(5.0, self._publish_waypoint_markers)

        self._pub_wheel = self.create_publisher(WheelMessage, "wheel_control", 10)
        self._approach_goal_xy: tuple[float, float] | None = None
        self._approach_start_time: float = 0.0
        self._approach_goal_theta: float = 0.0
        self._latest_scan: LaserScan | None = None
        self._scan_lock = threading.Lock()
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan_filtered", self._on_scan, 10
        )
        self._current_time_steps: int = MPPI_TIME_STEPS
        self._set_params_client = self.create_client(
            SetParameters, "/controller_server/set_parameters"
        )
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._amcl_params_client = self.create_client(
            SetParameters, "/amcl/set_parameters"
        )
        self.create_timer(0.05, self._on_approach_timer)

        self._hand_fb_list: list[HandMessage] = []
        self._hand_fb_lock = threading.Lock()
        self._sub_hand_fb = self.create_subscription(
            HandMessage, "hand_feedback", self._on_hand_feedback, 10
        )

        self._yagura_pos: PointStamped | None = None
        self._yagura_pos_1: PointStamped | None = None
        self._yagura_pos_lock = threading.Lock()
        self._sub_yagura = self.create_subscription(
            PointStamped, "yagura_position_0", self._on_yagura_pos, 10
        )
        self._sub_yagura_1 = self.create_subscription(
            PointStamped, "yagura_position_1", self._on_yagura_pos_1, 10
        )
        self._yagura_correction_applied: bool = False

        self._approach_done_event: threading.Event | None = None
        self._approach_success: bool = False

        self._ring_align_result: dict | None = None
        self._ring_align_lock = threading.Lock()
        self._pub_ring_align = self.create_publisher(String, "ring_align_cmd", 10)
        self._sub_ring_align = self.create_subscription(
            String, "ring_align_status", self._on_ring_align_status, 10
        )

        self._sequence_abort = threading.Event()
        self._sequence_thread: threading.Thread | None = None
        self._auto_seq_running: bool = False
        self._auto_seq_index: int = 0
        self._auto_seq_started_midway: bool = False

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = msg

    def _on_hand_feedback(self, msg: HandMessage) -> None:
        with self._hand_fb_lock:
            self._hand_fb_list.append(msg)
            if len(self._hand_fb_list) > 100:
                self._hand_fb_list = self._hand_fb_list[-50:]

    def _on_yagura_pos(self, msg: PointStamped) -> None:
        with self._yagura_pos_lock:
            self._yagura_pos = msg

    def _on_yagura_pos_1(self, msg: PointStamped) -> None:
        with self._yagura_pos_lock:
            self._yagura_pos_1 = msg

    def _on_ring_align_status(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._ring_align_lock:
            self._ring_align_result = result

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
            if self._state == "DIRECT_APPROACH":
                self._publish_wheel_rpms(0.0, 0.0, 0.0, 0.0)
                self._approach_goal_xy = None
            self._sequence_abort.set()
            self._auto_seq_running = False
            self._set_led("error_led", False)
            self._set_led("vgoal_led", False)
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
        self._approach_goal_xy = (x, y)
        self._approach_goal_theta = theta
        self._goal_retry_count = 0
        self._nav_failure_retry_count = 0

        wp = self._waypoints.get(self._current_goal_name, {})
        if wp.get("yagura_correction"):
            self._yagura_correction_applied = False

        if self._current_goal_name == "ring_pickup_1":
            self._set_mppi_critics(MPPI_RING_PICKUP_1)
        else:
            self._set_mppi_critics(MPPI_DEFAULT)

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
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
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
            # bt_navigator が lifecycle active に遷移するまで時間がかかるためリトライ回数を多めに設定
            _MAX_RETRIES = 15
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
            self._clear_local_costmap()
            self._clear_global_costmap()
            x, y, theta = self._pending_goal_xyz
            self._send_goal_impl(x, y, theta)

    def _result_cb(self, future) -> None:
        from action_msgs.msg import GoalStatus

        result = future.result()

        # yagura_approach のネスト航行完了: シーケンススレッドに通知して即リターン
        if self._approach_done_event is not None:
            self._approach_success = (result.status == GoalStatus.STATUS_SUCCEEDED)
            event = self._approach_done_event
            self._approach_done_event = None
            event.set()
            return

        # 直接アプローチによるキャンセルの場合は _on_approach_timer 側で制御する
        if result.status == GoalStatus.STATUS_CANCELED:
            if self._state in ("DIRECT_APPROACH", "SEQUENCE", "NAVIGATING"):
                return
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
        preconditions = wp.get("preconditions", [])
        on_arrive = wp.get("on_arrive", [])

        if preconditions or on_arrive:
            self._state = "SEQUENCE"
            self._sequence_abort.clear()
            t = threading.Thread(
                target=self._run_on_arrive_sequence,
                args=(preconditions, on_arrive, self._on_sequence_done),
                daemon=True,
            )
            self._sequence_thread = t
            t.start()
        else:
            self._on_sequence_done()

    def _check_preconditions(self, preconditions: list) -> bool:
        """preconditions を検証し、不一致なら自動修正する。成功で True、失敗で False。"""
        for pre in preconditions:
            if self._sequence_abort.is_set():
                return False

            target = pre.get("target", "")
            control_type = pre.get("control_type", "")
            expected_str = pre.get("expected", "")
            correct_action = pre.get("correct_action", "")
            correct_wait = pre.get("correct_wait", "")
            timeout = float(pre.get("timeout", 10.0))

            value_map = self._get_value_map(target, control_type)
            if value_map is None or expected_str not in value_map:
                self._abort_sequence_with_error(
                    f"precondition: invalid target/control_type/expected: "
                    f"{target}/{control_type}/{expected_str}"
                )
                return False

            expected_val = value_map[expected_str]

            # 最新の hand_feedback を確認
            with self._hand_fb_lock:
                msgs = list(self._hand_fb_list)
            if not msgs:
                self.get_logger().info(
                    f"precondition: hand_feedback 未受信、スキップ ({target}.{control_type})"
                )
                continue

            current_val = self._read_fb_field(msgs[-1], target, control_type)
            if current_val == expected_val:
                self.get_logger().info(
                    f"precondition OK: {target}.{control_type}={expected_str}"
                )
                continue

            # 不一致 → 自動修正
            self.get_logger().warn(
                f"precondition MISMATCH: {target}.{control_type} "
                f"expected={expected_str}, correcting with {correct_action}"
            )

            # hand_control コマンド送信
            cmd = {
                "type": "hand_control",
                "target": target,
                "control_type": control_type,
                "action": correct_action,
            }
            with self._hand_fb_lock:
                self._hand_fb_list.clear()
            self._pub_rx.publish(String(data=json.dumps(cmd)))

            # correct_wait の値を待つ
            wait_map = self._get_value_map(target, control_type)
            if wait_map is None or correct_wait not in wait_map:
                self._abort_sequence_with_error(
                    f"precondition: invalid correct_wait: "
                    f"{target}/{control_type}/{correct_wait}"
                )
                return False
            wait_val = wait_map[correct_wait]

            deadline = time.monotonic() + timeout
            interval = 0.05
            while time.monotonic() < deadline:
                if self._sequence_abort.is_set():
                    return False
                with self._hand_fb_lock:
                    fb_msgs = list(self._hand_fb_list)
                for fb in fb_msgs:
                    if self._read_fb_field(fb, target, control_type) == wait_val:
                        self.get_logger().info(
                            f"precondition corrected: {target}.{control_type}={correct_wait}"
                        )
                        break
                else:
                    time.sleep(interval)
                    continue
                break
            else:
                self._abort_sequence_with_error(
                    f"precondition correction timeout: {target}.{control_type}={correct_wait} "
                    f"({timeout}s)"
                )
                return False

        return True

    def _run_on_arrive_sequence(self, preconditions: list, actions: list, done_callback) -> None:
        if preconditions and self._auto_seq_started_midway:
            self._auto_seq_started_midway = False
            if not self._check_preconditions(preconditions):
                return

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
                with self._hand_fb_lock:
                    self._hand_fb_list.clear()
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
            elif action_type == "yagura_approach":
                approach_dist = float(act.get("approach_dist", 0.35))
                timeout = float(act.get("timeout", 30.0))

                # キャッシュされた櫓位置を取得
                with self._yagura_pos_lock:
                    yagura_pos = self._yagura_pos
                if yagura_pos is None:
                    self._abort_sequence_with_error("yagura_approach: yagura_position_0 未受信")
                    return

                # 櫓位置を map 座標に変換
                try:
                    tf = self._tf_buffer.lookup_transform(
                        "map", yagura_pos.header.frame_id, rclpy.time.Time()
                    )
                except Exception as e:
                    self._abort_sequence_with_error(f"yagura_approach: TF失敗: {e}")
                    return

                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                tx, ty = tf.transform.translation.x, tf.transform.translation.y
                px = cos_y * yagura_pos.point.x - sin_y * yagura_pos.point.y + tx
                py = sin_y * yagura_pos.point.x + cos_y * yagura_pos.point.y + ty

                hand_offset_x = float(act.get("hand_offset_x", 0.35))
                hand_offset_y = float(act.get("hand_offset_y", 0.14))
                # 赤コートでは180°回転のため、X方向オフセットを反転し北向きにアプローチ
                court_sign = -1.0 if self._current_court == "red" else 1.0
                goal_x = px + court_sign * hand_offset_x
                goal_y = py - hand_offset_y
                goal_theta = math.pi / 2.0 if self._current_court == "red" else -math.pi / 2.0

                # ネスト航行を開始してシーケンススレッドで完了を待つ
                done_event = threading.Event()
                self._approach_done_event = done_event
                self._approach_success = False
                saved_goal_name = self._current_goal_name
                self._current_goal_name = "__yagura_approach__"
                self._state = "NAVIGATING"
                self._pending_goal_xyz = (goal_x, goal_y, goal_theta)
                self._goal_retry_count = 0
                self._nav_failure_retry_count = 0
                self._send_goal_impl(goal_x, goal_y, goal_theta)

                done_event.wait(timeout=timeout)
                self._current_goal_name = saved_goal_name
                self._state = "SEQUENCE"

                if not self._approach_success:
                    self._abort_sequence_with_error("yagura_approach: ナビゲーション失敗またはタイムアウト")
                    return

            elif action_type == "wait_actuator":
                target = act.get("target", "")
                control_type = act.get("control_type", "")
                value_str = act.get("value", "")
                timeout = float(act.get("timeout", 10.0))

                value_map = self._get_value_map(target, control_type)
                if value_map is None or value_str not in value_map:
                    self._abort_sequence_with_error(
                        f"wait_actuator: invalid target/control_type/value: "
                        f"{target}/{control_type}/{value_str}"
                    )
                    return
                expected = value_map[value_str]

                grip_fail_val = None
                if target.startswith("ring_"):
                    grip_fail_val = _RING_STATE_MAP["grip_fail"]

                retry_on_grip_fail = act.get("retry_on_grip_fail", False)
                max_retries = int(act.get("max_retries", 0))
                attempt = 0
                interval = 0.05

                while True:
                    deadline = time.monotonic() + timeout
                    result = None

                    while time.monotonic() < deadline:
                        if self._sequence_abort.is_set():
                            return
                        with self._hand_fb_lock:
                            msgs = list(self._hand_fb_list)
                        found = False
                        grip_fail_detected = False
                        for fb in msgs:
                            if self._read_fb_field(fb, target, control_type) == expected:
                                found = True
                                break
                            if grip_fail_val is not None:
                                if self._read_fb_field(fb, target, "state") == grip_fail_val:
                                    grip_fail_detected = True
                                    break
                        if found:
                            result = "ok"
                            break
                        if grip_fail_detected:
                            result = "grip_fail"
                            break
                        time.sleep(interval)
                    else:
                        result = "timeout"

                    if result == "ok":
                        break

                    if result == "grip_fail":
                        if retry_on_grip_fail and attempt < max_retries:
                            attempt += 1
                            self.get_logger().warn(
                                f"wait_actuator: {target} grip_fail, "
                                f"retry {attempt}/{max_retries}"
                            )
                            close_done_val = _RING_STATE_MAP["close_done"]
                            with self._hand_fb_lock:
                                self._hand_fb_list.clear()
                            close_deadline = time.monotonic() + timeout
                            close_ok = False
                            while time.monotonic() < close_deadline:
                                if self._sequence_abort.is_set():
                                    return
                                with self._hand_fb_lock:
                                    close_msgs = list(self._hand_fb_list)
                                for fb in close_msgs:
                                    if self._read_fb_field(fb, target, "state") == close_done_val:
                                        close_ok = True
                                        break
                                if close_ok:
                                    break
                                time.sleep(interval)
                            if not close_ok:
                                self._abort_sequence_with_error(
                                    f"wait_actuator: {target} retry close_done timeout"
                                )
                                return
                            cmd = {
                                "type": "hand_control",
                                "target": target,
                                "control_type": "state",
                                "action": "open",
                            }
                            with self._hand_fb_lock:
                                self._hand_fb_list.clear()
                            self._pub_rx.publish(String(data=json.dumps(cmd)))
                            continue
                        self.get_logger().warn(
                            f"wait_actuator: {target} grip_fail, continuing sequence"
                            + (f" (after {attempt} retries)" if attempt > 0 else "")
                        )
                        break

                    if result == "timeout":
                        self._abort_sequence_with_error(
                            f"wait_actuator: {target}.{control_type}={value_str} "
                            f"timeout ({timeout}s)"
                        )
                        return

            elif action_type == "early_depart":
                delay = float(act.get("delay", 0.0))
                if delay > 0:
                    elapsed = 0.0
                    interval = 0.05
                    while elapsed < delay:
                        if self._sequence_abort.is_set():
                            return
                        time.sleep(interval)
                        elapsed += interval
                self.get_logger().info(
                    f"early_depart: departing (delay={delay}s)"
                )
                done_callback()
                done_callback = lambda: None

            elif action_type == "coord_align":
                self._run_coord_align(act)

            elif action_type == "ring_align":
                self._run_ring_align(act)

        done_callback()

    def _run_coord_align(self, act: dict) -> None:
        """odom座標ベースでウェイポイント位置にP制御で微調整する。"""
        tolerance = float(act.get("tolerance", 0.01))
        timeout = float(act.get("timeout", 3.0))
        kp = float(act.get("kp", 1.0))
        max_speed = float(act.get("max_speed", 0.08))
        settle_target = int(act.get("settle_frames", 3))

        goal_name = self._current_goal_name
        if goal_name is None or goal_name not in self._waypoints:
            self.get_logger().warn("coord_align: no current waypoint, skipping")
            return

        wp = self._waypoints[goal_name]
        target_x = wp["x"]
        target_y = wp["y"]

        if self._current_court == "red":
            target_x = -target_x

        self.get_logger().info(
            f"coord_align: target=({target_x:.3f}, {target_y:.3f}), "
            f"tolerance={tolerance}m, timeout={timeout}s"
        )

        settle_count = 0
        interval = 0.05
        deadline = time.monotonic() + timeout
        pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        try:
            while time.monotonic() < deadline:
                if self._sequence_abort.is_set():
                    return

                try:
                    tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    cur_x = tf.transform.translation.x
                    cur_y = tf.transform.translation.y
                except Exception:
                    time.sleep(interval)
                    continue

                dx = target_x - cur_x
                dy = target_y - cur_y
                dist = math.hypot(dx, dy)

                if dist < tolerance:
                    settle_count += 1
                    pub_cmd_vel.publish(Twist())
                    if settle_count >= settle_target:
                        self.get_logger().info(
                            f"coord_align: aligned (err={dist:.4f}m)"
                        )
                        return
                else:
                    settle_count = 0
                    vx = max(-max_speed, min(max_speed, kp * dx))
                    vy = max(-max_speed, min(max_speed, kp * dy))

                    # map座標の速度をbase_link座標に変換
                    q = tf.transform.rotation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    cos_y = math.cos(-yaw)
                    sin_y = math.sin(-yaw)
                    vx_body = vx * cos_y - vy * sin_y
                    vy_body = vx * sin_y + vy * cos_y

                    twist = Twist()
                    twist.linear.x = vx_body
                    twist.linear.y = vy_body
                    pub_cmd_vel.publish(twist)

                time.sleep(interval)

            self.get_logger().warn(
                f"coord_align: timeout (err={dist:.4f}m)"
            )
        finally:
            pub_cmd_vel.publish(Twist())

    def _run_ring_align(self, act: dict) -> None:
        """カメラベースのリングアライメントを実行する。"""
        color = act.get("color", "any")
        timeout = float(act.get("timeout", 5.0))

        with self._ring_align_lock:
            self._ring_align_result = None

        cmd = json.dumps({"action": "start", "color": color, "timeout": timeout})
        self._pub_ring_align.publish(String(data=cmd))
        self.get_logger().info(f"ring_align: start (color={color}, timeout={timeout}s)")

        deadline = time.monotonic() + timeout + 2.0
        interval = 0.05
        while time.monotonic() < deadline:
            if self._sequence_abort.is_set():
                self._pub_ring_align.publish(
                    String(data=json.dumps({"action": "cancel"}))
                )
                return
            with self._ring_align_lock:
                result = self._ring_align_result
            if result is not None:
                if result.get("success"):
                    self.get_logger().info(
                        f"ring_align success: offset=("
                        f"{result.get('offset_x', 0):.3f}, "
                        f"{result.get('offset_y', 0):.3f})m"
                    )
                else:
                    self.get_logger().warn(
                        f"ring_align failed: {result.get('message', 'unknown')}"
                    )
                return
            time.sleep(interval)

        self._pub_ring_align.publish(
            String(data=json.dumps({"action": "cancel"}))
        )
        self.get_logger().warn("ring_align: routing timeout")

    def _get_value_map(self, target: str, control_type: str) -> dict | None:
        if target.startswith("yagura_"):
            return _YAGURA_POS_MAP if control_type == "pos" else _YAGURA_STATE_MAP if control_type == "state" else None
        if target.startswith("ring_"):
            return _RING_POS_MAP if control_type == "pos" else _RING_STATE_MAP if control_type == "state" else None
        return None

    def _read_fb_field(self, fb: HandMessage, target: str, control_type: str) -> int:
        mech = getattr(fb, target)
        return getattr(mech, control_type)

    def _set_led(self, target: str, on: bool) -> None:
        self._pub_rx.publish(String(data=json.dumps({
            "type": "hand_control",
            "target": target,
            "control_type": "state",
            "action": "on" if on else "off",
        })))

    def _abort_sequence_with_error(self, message: str) -> None:
        self.get_logger().error(message)
        self._auto_seq_running = False
        self._state = "AUTO_IDLE"
        self._set_led("error_led", True)
        self._pub_tx.publish(String(data=json.dumps({
            "nav_status": "error",
            "waypoint": self._current_goal_name,
            "message": message,
            "seq_index": self._auto_seq_index,
            "seq_total": len(self._auto_seq_names),
        })))

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
                need_back_off = ("pickup" in self._current_goal_name or "release" in self._current_goal_name) and "_via" not in self._current_goal_name
                if need_back_off:
                    t = threading.Thread(
                        target=self._back_off_and_advance,
                        daemon=True,
                    )
                    t.start()
                else:
                    self._clear_local_costmap()
                    self._clear_global_costmap()
                    self._advance_auto_sequence()
                return
            else:
                self._auto_seq_running = False
                self._state = "AUTO_IDLE"
                self._set_led("vgoal_led", True)
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
        # 赤コートは青コートに対して点対称（180°回転）: X反転 + 向き反転
        if self._current_court == "red":
            return -x, y, theta + math.pi
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

        self._set_led("error_led", False)
        self._set_led("vgoal_led", False)

        self._auto_seq_running = True
        self._auto_seq_index = from_index
        self._auto_seq_started_midway = from_index > 0
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
            time.sleep(1.0)

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

    def _publish_initial_pose_from_tf(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"initialpose from TF failed: {e}")
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = (
            self.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
        ).to_msg()
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.orientation.z = tf.transform.rotation.z
        msg.pose.pose.orientation.w = tf.transform.rotation.w
        msg.pose.covariance[0] = 0.10   # x
        msg.pose.covariance[7] = 0.10   # y
        msg.pose.covariance[35] = 0.02  # yaw
        self._pub_initial_pose.publish(msg)

    def _handle_stop_auto(self) -> None:
        if self._state == "DIRECT_APPROACH":
            self._publish_wheel_rpms(0.0, 0.0, 0.0, 0.0)
            self._approach_goal_xy = None
        else:
            self._cancel_goal()
        self._sequence_abort.set()
        self._auto_seq_running = False
        self._state = "AUTO_IDLE"
        self._set_led("error_led", False)
        self._set_led("vgoal_led", False)

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
                and self._state in ("NAVIGATING", "DIRECT_APPROACH", "SEQUENCE")
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

    def _on_approach_timer(self) -> None:
        if self._state not in ("NAVIGATING", "DIRECT_APPROACH"):
            return

        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return

        if self._approach_goal_xy is None:
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        goal_x, goal_y = self._approach_goal_xy
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)

        if self._state == "NAVIGATING":
            self._try_yagura_correction(robot_x, robot_y)

            if dist <= DIRECT_APPROACH_DIST:
                self.get_logger().info(
                    f"Direct approach activated (dist={dist:.3f}m)"
                )
                self._state = "DIRECT_APPROACH"
                self._approach_start_time = time.monotonic()
                self._set_mppi_critics(MPPI_DEFAULT)
                if self._goal_handle:
                    self._goal_handle.cancel_goal_async()
                    self._goal_handle = None
                self._set_amcl_direct_approach_params()
                self._publish_initialpose_once()
                self._fire_pre_arrive()
                self._run_direct_approach_step(tf, goal_x, goal_y, dist)
                return

            return

        self._run_direct_approach_step(tf, goal_x, goal_y, dist)

    def _fire_pre_arrive(self) -> None:
        wp = self._waypoints.get(self._current_goal_name, {})
        pre_arrive = wp.get("pre_arrive", [])
        if not pre_arrive:
            return
        self.get_logger().info(
            f"pre_arrive: firing {len(pre_arrive)} command(s)"
        )
        for act in pre_arrive:
            if act.get("action") == "hand_control":
                cmd = {
                    "type": "hand_control",
                    "target": act.get("target", ""),
                    "control_type": act.get("control_type", ""),
                    "action": act.get("action_value", ""),
                }
                self._pub_rx.publish(String(data=json.dumps(cmd)))

    def _run_direct_approach_step(self, tf, goal_x: float, goal_y: float, dist: float) -> None:
        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)

        dx = goal_x - robot_x
        dy = goal_y - robot_y

        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        dx_local = cos_yaw * dx - sin_yaw * dy
        dy_local = sin_yaw * dx + cos_yaw * dy

        yaw_error = math.atan2(
            math.sin(self._approach_goal_theta - yaw),
            math.cos(self._approach_goal_theta - yaw),
        )

        approach_angle = math.atan2(dy_local, dx_local)
        lidar_dist = self._lidar_forward_min_dist(approach_angle)

        wp = self._waypoints.get(self._current_goal_name, {})
        goal_dist_tol = float(wp.get("tolerance_dist", GOAL_REACHED_DIST))
        goal_yaw_tol = float(wp.get("tolerance_yaw", YAW_TOLERANCE))

        amcl_reached = dist <= goal_dist_tol and abs(yaw_error) <= goal_yaw_tol
        lidar_reached = lidar_dist is not None and lidar_dist <= LIDAR_STOP_DIST
        elapsed = time.monotonic() - self._approach_start_time
        timeout_reached = elapsed >= DIRECT_APPROACH_TIMEOUT

        if amcl_reached or lidar_reached or timeout_reached:
            if amcl_reached:
                self.get_logger().info(
                    f"Goal reached (direct approach, dist={dist:.3f}m, yaw_err={yaw_error:.3f})"
                )
            elif lidar_reached:
                self.get_logger().info(
                    f"Goal reached by LiDAR (forward_dist={lidar_dist:.3f}m)"
                )
            else:
                self.get_logger().warn(
                    f"Goal reached by timeout ({elapsed:.1f}s elapsed, dist={dist:.3f}m)"
                )
            self._publish_wheel_rpms(0.0, 0.0, 0.0, 0.0)
            self._approach_goal_xy = None
            self._restore_amcl_default_params()

            preconditions = wp.get("preconditions", [])
            on_arrive = wp.get("on_arrive", [])
            if preconditions or on_arrive:
                self._state = "SEQUENCE"
                self._sequence_abort.clear()
                t = threading.Thread(
                    target=self._run_on_arrive_sequence,
                    args=(preconditions, on_arrive, self._on_sequence_done),
                    daemon=True,
                )
                self._sequence_thread = t
                t.start()
            else:
                self._on_sequence_done()
            return

        vx = KP_POS * dx_local
        vy = KP_POS * dy_local
        speed = math.hypot(vx, vy)
        if speed > DIRECT_APPROACH_SPEED:
            vx = vx / speed * DIRECT_APPROACH_SPEED
            vy = vy / speed * DIRECT_APPROACH_SPEED

        omega = max(-MAX_OMEGA, min(MAX_OMEGA, KP_YAW * yaw_error))

        vy_c = vy * 1.35
        v_fl = +vx - vy_c - G * omega
        v_fr = -vx - vy_c - G * omega
        v_rl = +vx + vy_c - G * omega
        v_rr = -vx + vy_c - G * omega

        rpms = [v * MS_TO_RPM for v in (v_fl, v_fr, v_rl, v_rr)]
        max_abs = max(abs(r) for r in rpms)

        if max_abs > MAX_RPM:
            scale = MAX_RPM / max_abs
            rpms = [r * scale for r in rpms]

        self._publish_wheel_rpms(*rpms)

    def _lidar_forward_min_dist(self, approach_angle_base_link: float) -> float | None:
        """approach_angle_base_link 方向の前方 ±LIDAR_FORWARD_ANGLE 内の最小距離を返す"""
        with self._scan_lock:
            scan = self._latest_scan
        if scan is None:
            return None

        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment

        diff = np.arctan2(np.sin(angles - approach_angle_base_link),
                          np.cos(angles - approach_angle_base_link))
        valid = (np.abs(diff) <= LIDAR_FORWARD_ANGLE) & np.isfinite(ranges) & \
                (ranges >= scan.range_min) & (ranges <= scan.range_max)

        if not np.any(valid):
            return None
        return float(np.min(ranges[valid]))

    def _clear_local_costmap(self) -> None:
        if not self._clear_costmap_client.service_is_ready():
            self.get_logger().warn("clear_costmap service not ready, skipping")
            return
        self._clear_costmap_client.call_async(ClearEntireCostmap.Request())

    def _clear_global_costmap(self) -> None:
        if not self._clear_global_costmap_client.service_is_ready():
            self.get_logger().warn("clear_global_costmap service not ready, skipping")
            return
        self._clear_global_costmap_client.call_async(ClearEntireCostmap.Request())

    def _compute_ring_pickup_back_off_direction(self) -> float:
        """ring_pickup ウェイポイントの back-off 方向（map 座標系）を返す。
        blue コートは南西（-3π/4）、red コートは南東（-π/4）。"""
        if self._current_court == "red":
            return -math.pi / 4.0
        return -3.0 * math.pi / 4.0

    def _compute_release_back_off_direction(self) -> float:
        """release ウェイポイントの back-off 方向（map 座標系）を返す。
        blue コートは東（0 rad）、red コートは西（π rad）。"""
        if self._current_court == "red":
            return math.pi
        return 0.0

    def _compute_back_off_direction(self) -> float | None:
        """LiDAR 近傍障害物の加重平均方向から、逆方向（base_link 座標系）を算出する。

        inscribed_radius + マージン（0.5m）以内の全障害物点を 1/r² で加重平均し、
        その重心と逆方向を返す。複数の壁に挟まれている場合は斜めに離れる。
        """
        _NEAR_THRESHOLD = 0.5
        with self._scan_lock:
            scan = self._latest_scan
        if scan is None:
            return None

        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & \
                (ranges <= scan.range_max) & (ranges < _NEAR_THRESHOLD)

        if not np.any(valid):
            return None

        r_valid = ranges[valid]
        a_valid = angles[valid]
        w = 1.0 / (r_valid * r_valid)
        wx = float(np.sum(w * np.cos(a_valid)))
        wy = float(np.sum(w * np.sin(a_valid)))

        obstacle_angle = math.atan2(wy, wx)
        return obstacle_angle + math.pi

    def _back_off_and_advance(self) -> None:
        """壁密着 waypoint から後退し、コストマップクリア後に次の waypoint へ進む。

        LiDAR 近傍障害物の加重平均逆方向に後退して
        inscribed ゾーンを脱出してから Nav2 ゴールを送信する。
        """
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            self.get_logger().warn("Back-off: TF not available, skipping back-off")
            self._clear_local_costmap()
            self._clear_global_costmap()
            self._advance_auto_sequence()
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)

        if "release" in self._current_goal_name:
            back_dir = self._compute_release_back_off_direction()
        elif "ring_pickup" in self._current_goal_name:
            back_dir = self._compute_ring_pickup_back_off_direction()
        else:
            back_off_local = self._compute_back_off_direction()
            if back_off_local is not None:
                back_dir = yaw + back_off_local
            else:
                wp = self._waypoints.get(self._current_goal_name, {})
                _, _, theta = self._apply_court_transform(
                    wp.get("x", 0), wp.get("y", 0), wp.get("theta", 0)
                )
                back_dir = theta + math.pi

        target_x = robot_x + WALL_BACK_OFF_DIST * math.cos(back_dir)
        target_y = robot_y + WALL_BACK_OFF_DIST * math.sin(back_dir)

        self.get_logger().info(
            f"Back-off from wall: ({robot_x:.2f},{robot_y:.2f}) → "
            f"({target_x:.2f},{target_y:.2f})"
        )

        # back-off 開始前に次のゴールを Nav2 に送信（プランニングを並行実行）
        self._clear_local_costmap()
        self._clear_global_costmap()
        self._advance_auto_sequence()

        deadline = time.monotonic() + BACK_OFF_TIMEOUT
        dt = 0.05

        while time.monotonic() < deadline:
            if self._sequence_abort.is_set():
                self._publish_wheel_rpms(0.0, 0.0, 0.0, 0.0)
                return

            try:
                tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception:
                time.sleep(dt)
                continue

            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            dx = target_x - rx
            dy = target_y - ry
            dist = math.hypot(dx, dy)

            if dist < BACK_OFF_REACHED_DIST:
                self.get_logger().info(f"Back-off complete (dist={dist:.3f}m)")
                break

            cur_qz = tf.transform.rotation.z
            cur_qw = tf.transform.rotation.w
            current_yaw = 2.0 * math.atan2(cur_qz, cur_qw)
            cos_yaw = math.cos(-current_yaw)
            sin_yaw = math.sin(-current_yaw)
            dx_local = cos_yaw * dx - sin_yaw * dy
            dy_local = sin_yaw * dx + cos_yaw * dy

            vx = KP_POS * dx_local
            vy = KP_POS * dy_local
            speed = math.hypot(vx, vy)
            if speed > BACK_OFF_SPEED:
                vx = vx / speed * BACK_OFF_SPEED
                vy = vy / speed * BACK_OFF_SPEED

            vy_c = vy * 1.35
            v_fl = +vx - vy_c
            v_fr = -vx - vy_c
            v_rl = +vx + vy_c
            v_rr = -vx + vy_c
            rpms = [v * MS_TO_RPM for v in (v_fl, v_fr, v_rl, v_rr)]
            max_abs = max(abs(r) for r in rpms)
            if max_abs > MAX_RPM:
                scale = MAX_RPM / max_abs
                rpms = [r * scale for r in rpms]

            self._publish_wheel_rpms(*rpms)
            time.sleep(dt)
        else:
            self.get_logger().warn("Back-off timeout, proceeding anyway")

        self._publish_wheel_rpms(0.0, 0.0, 0.0, 0.0)
        self._publish_initial_pose_from_tf()
        self._clear_local_costmap()
        self._clear_global_costmap()

    def _set_mppi_time_steps(self, steps: int) -> None:
        if steps == self._current_time_steps:
            return
        self._current_time_steps = steps

        if not self._set_params_client.service_is_ready():
            self.get_logger().warn("controller_server set_parameters service not ready")
            return

        param = Parameter()
        param.name = "FollowPath.time_steps"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=steps
        )
        req = SetParameters.Request(parameters=[param])
        future = self._set_params_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"MPPI time_steps → {steps}")
        )

    def _set_mppi_critics(self, params: dict) -> None:
        if not self._set_params_client.service_is_ready():
            self.get_logger().warn("controller_server set_parameters service not ready")
            return
        param_list = []
        for name, value in params.items():
            p = Parameter()
            p.name = name
            if isinstance(value, bool):
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=value
                )
            elif isinstance(value, int):
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=value
                )
            else:
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                )
            param_list.append(p)
        req = SetParameters.Request(parameters=param_list)
        future = self._set_params_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"MPPI critics updated: {list(params.keys())}")
        )

    def _try_yagura_correction(self, robot_x: float, robot_y: float) -> None:
        if self._yagura_correction_applied:
            return

        wp = self._waypoints.get(self._current_goal_name, {})
        yc = wp.get("yagura_correction")
        if yc is None:
            return
        if self._approach_goal_xy is None or self._pending_goal_xyz is None:
            return

        goal_x, goal_y = self._approach_goal_xy
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_goal > YAGURA_CORRECTION_DIST:
            return

        with self._yagura_pos_lock:
            yp0 = self._yagura_pos
            yp1 = self._yagura_pos_1

        detected = []
        for yp in (yp0, yp1):
            if yp is None:
                continue
            try:
                tf = self._tf_buffer.lookup_transform(
                    "map", yp.header.frame_id, rclpy.time.Time()
                )
            except Exception:
                continue
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            px = cos_y * yp.point.x - sin_y * yp.point.y + tx
            py = sin_y * yp.point.x + cos_y * yp.point.y + ty
            detected.append((px, py))

        if not detected:
            return

        wp_x, wp_y, wp_theta = wp["x"], wp["y"], wp["theta"]
        wp_x, wp_y, wp_theta = self._apply_court_transform(wp_x, wp_y, wp_theta)
        cos_t = math.cos(wp_theta)
        sin_t = math.sin(wp_theta)

        # 赤コートは180°回転（点対称）のため、ロボットが逆向きになり body X 方向が反転する
        court_sign = -1.0 if self._current_court == "red" else 1.0

        expected = []
        for off in yc.get("offsets", []):
            x_body, y_body = court_sign * off[0], off[1]
            ex = wp_x + cos_t * x_body - sin_t * y_body
            ey = wp_y + sin_t * x_body + cos_t * y_body
            expected.append((ex, ey))

        if not expected:
            return

        corrections = []
        used_expected = set()
        for dx, dy in detected:
            best_idx = -1
            best_dist = float("inf")
            for i, (ex, ey) in enumerate(expected):
                if i in used_expected:
                    continue
                d = math.hypot(dx - ex, dy - ey)
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            if best_idx >= 0 and best_dist < 1.0:
                ex, ey = expected[best_idx]
                corrections.append((dx - ex, dy - ey))
                used_expected.add(best_idx)

        if not corrections:
            return

        corr_x = sum(c[0] for c in corrections) / len(corrections)
        corr_y = sum(c[1] for c in corrections) / len(corrections)
        corr_mag = math.hypot(corr_x, corr_y)

        if corr_mag > YAGURA_CORRECTION_MAX:
            self.get_logger().warn(
                f"Yagura correction too large ({corr_mag:.3f}m), skipping"
            )
            self._yagura_correction_applied = True
            return

        orig_x, orig_y, orig_theta = self._pending_goal_xyz
        new_x = orig_x + corr_x
        new_y = orig_y + corr_y

        self.get_logger().info(
            f"Yagura correction applied: ({corr_x:+.3f}, {corr_y:+.3f})m "
            f"from {len(corrections)} yagura(s), "
            f"goal ({orig_x:.3f},{orig_y:.3f}) → ({new_x:.3f},{new_y:.3f})"
        )

        self._yagura_correction_applied = True
        self._pending_goal_xyz = (new_x, new_y, orig_theta)
        self._approach_goal_xy = (new_x, new_y)

        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_retry_count = 0
        self._nav_failure_retry_count = 0
        self._send_goal_impl(new_x, new_y, orig_theta)

    def _set_amcl_params(self, params: dict) -> None:
        if not self._amcl_params_client.service_is_ready():
            self.get_logger().warn("AMCL set_parameters service not ready")
            return
        param_list = []
        for name, value in params.items():
            p = Parameter()
            p.name = name
            if isinstance(value, int):
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=value
                )
            else:
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                )
            param_list.append(p)
        req = SetParameters.Request(parameters=param_list)
        future = self._amcl_params_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"AMCL params updated: {list(params.keys())}")
        )

    def _set_amcl_direct_approach_params(self) -> None:
        self._set_amcl_params({
            "laser_max_range": 3.0,
            "laser_likelihood_max_dist": 1.0,
            "z_hit": 0.95,
            "z_rand": 0.02,
            "z_short": 0.02,
            "z_max": 0.01,
            "sigma_hit": 0.08,
            "max_beams": 120,
            "update_min_d": 0.01,
            "update_min_a": 0.02,
        })

    def _restore_amcl_default_params(self) -> None:
        self._set_amcl_params({
            "laser_max_range": 100.0,
            "laser_likelihood_max_dist": 2.0,
            "z_hit": 0.85,
            "z_rand": 0.05,
            "z_short": 0.05,
            "z_max": 0.05,
            "sigma_hit": 0.15,
            "max_beams": 60,
            "update_min_d": 0.05,
            "update_min_a": 0.10,
        })

    def _publish_initialpose_once(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.orientation.z = tf.transform.rotation.z
        msg.pose.pose.orientation.w = tf.transform.rotation.w
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        self._pub_initial_pose.publish(msg)
        self.get_logger().info("Published initialpose for direct approach re-localization")

    def _publish_wheel_rpms(
        self, fl: float, fr: float, rl: float, rr: float
    ) -> None:
        msg = WheelMessage()
        msg.m3508_rpms.fl = fl
        msg.m3508_rpms.fr = fr
        msg.m3508_rpms.rl = rl
        msg.m3508_rpms.rr = rr
        self._pub_wheel.publish(msg)

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
