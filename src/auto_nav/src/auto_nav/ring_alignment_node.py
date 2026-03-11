"""
ring_alignment_node — カメラによるリング自動位置調整

USB Webカメラでリングを検出し、ビジュアルサーボ（P制御）で
ロボットのXY位置をリング中心に合わせる。

トピック:
  ring_align_cmd     (sub) 開始/キャンセル指示 (std_msgs/String, JSON)
  ring_align_status  (pub) 結果報告 (std_msgs/String, JSON)
  /cmd_vel           (pub) 位置調整用速度指令
  ring_align_debug   (pub) デバッグ画像 (sensor_msgs/Image)

cmd JSON:
  {"action": "start", "color": "any", "timeout": 5.0}
  {"action": "cancel"}

status JSON:
  {"success": true, "message": "aligned", "offset_x": 0.001, "offset_y": 0.002}
  {"success": false, "message": "ring_not_found"}
"""

import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

_LOOP_HZ = 20.0
_CAMERA_WARMUP_FRAMES = 5
_MAX_MISS_FRAMES = 20
_MIN_CONTOUR_AREA = 500
_MIN_CIRCULARITY = 0.6


class RingAlignmentNode(Node):
    def __init__(self) -> None:
        super().__init__("ring_alignment_node")

        self.declare_parameter("camera_device", 0)
        self.declare_parameter("ring_diameter", 0.08)
        self.declare_parameter("align_tolerance", 0.005)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("max_speed", 0.05)
        self.declare_parameter("settle_frames", 5)
        self.declare_parameter("target_offset_x_px", 0)
        self.declare_parameter("target_offset_y_px", 0)
        # カメラ座標→ロボット座標変換
        # カメラが前方斜め下向き: 画像Y下方向=ロボット前進(+X), 画像X右方向=ロボット左(-Y)
        self.declare_parameter("cam_x_to_robot_axis", "y")
        self.declare_parameter("cam_x_to_robot_sign", -1.0)
        self.declare_parameter("cam_y_to_robot_axis", "x")
        self.declare_parameter("cam_y_to_robot_sign", 1.0)
        self.declare_parameter("image_flip", False)

        self._pub_status = self.create_publisher(String, "ring_align_status", 10)
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_debug = self.create_publisher(Image, "ring_align_debug", 1)

        self.create_subscription(String, "ring_align_cmd", self._on_cmd, 10)

        self._color_profiles = self._load_color_profiles()
        self._align_thread = None
        self._cancel_event = threading.Event()
        self._active = False

        self.get_logger().info("ring_alignment_node ready")

    def _load_color_profiles(self) -> dict:
        share_dir = get_package_share_directory("auto_nav")
        config_path = os.path.join(share_dir, "config", "ring_colors.yaml")
        try:
            with open(config_path) as f:
                data = yaml.safe_load(f)
            profiles = {}
            for name, info in data.get("ring_colors", {}).items():
                ranges = []
                for r in info.get("ranges", []):
                    lower = np.array([r["h_min"], r["s_min"], r["v_min"]], dtype=np.uint8)
                    upper = np.array([r["h_max"], r["s_max"], r["v_max"]], dtype=np.uint8)
                    ranges.append((lower, upper))
                profiles[name] = ranges
            self.get_logger().info(f"loaded color profiles: {list(profiles.keys())}")
            return profiles
        except Exception as e:
            self.get_logger().error(f"failed to load ring_colors.yaml: {e}")
            return {}

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"invalid JSON: {msg.data}")
            return

        action = cmd.get("action", "")

        if action == "cancel":
            if self._active:
                self._cancel_event.set()
                self.get_logger().info("alignment cancel requested")
            return

        if action == "start":
            if self._active:
                self.get_logger().warn("alignment already active, ignoring")
                return
            color = cmd.get("color", "any")
            timeout = float(cmd.get("timeout", 5.0))
            self._cancel_event.clear()
            self._align_thread = threading.Thread(
                target=self._run_alignment, args=(color, timeout), daemon=True
            )
            self._align_thread.start()

    def _publish_status(self, success: bool, message: str,
                        offset_x: float = 0.0, offset_y: float = 0.0) -> None:
        status = {
            "success": success,
            "message": message,
            "offset_x": round(offset_x, 4),
            "offset_y": round(offset_y, 4),
        }
        self._pub_status.publish(String(data=json.dumps(status)))

    def _stop_robot(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _detect_ring(self, frame: np.ndarray, color_ranges: list) -> tuple:
        """リングを検出し、(center_x, center_y, radius) を返す。未検出時は None。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in color_ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _MIN_CONTOUR_AREA:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < _MIN_CIRCULARITY:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0
            if aspect < 0.6:
                continue

            if area > best_area:
                best_area = area
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                best = (cx, cy, radius, cnt)

        if best is None:
            self._publish_debug_image(frame, mask, None)
            return None

        cx, cy, radius, cnt = best
        self._publish_debug_image(frame, mask, (cx, cy, radius))
        return (cx, cy, radius)

    def _publish_debug_image(self, frame: np.ndarray, mask: np.ndarray,
                             detection: tuple | None) -> None:
        if self._pub_debug.get_subscription_count() == 0:
            return

        debug = frame.copy()

        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0
        mask_colored[:, :, 2] = 0
        debug = cv2.addWeighted(debug, 0.7, mask_colored, 0.3, 0)

        h, w = frame.shape[:2]
        target_x = w // 2 + self.get_parameter("target_offset_x_px").value
        target_y = h // 2 + self.get_parameter("target_offset_y_px").value
        cv2.drawMarker(debug, (target_x, target_y), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        if detection is not None:
            cx, cy, radius = detection
            cv2.circle(debug, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
            cv2.circle(debug, (int(cx), int(cy)), 3, (0, 0, 255), -1)
            cv2.line(debug, (target_x, target_y), (int(cx), int(cy)), (255, 0, 0), 1)

        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera"
        img_msg.height = debug.shape[0]
        img_msg.width = debug.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.step = debug.shape[1] * 3
        img_msg.data = debug.tobytes()
        self._pub_debug.publish(img_msg)

    def _pixel_to_robot(self, dx_px: float, dy_px: float, m_per_px: float) -> tuple:
        """画像座標オフセット→ロボット座標オフセット (robot_vx, robot_vy)"""
        cam_x_axis = self.get_parameter("cam_x_to_robot_axis").value
        cam_x_sign = self.get_parameter("cam_x_to_robot_sign").value
        cam_y_axis = self.get_parameter("cam_y_to_robot_axis").value
        cam_y_sign = self.get_parameter("cam_y_to_robot_sign").value

        cam_dx_m = dx_px * m_per_px
        cam_dy_m = dy_px * m_per_px

        robot = {"x": 0.0, "y": 0.0}
        robot[cam_x_axis] += cam_x_sign * cam_dx_m
        robot[cam_y_axis] += cam_y_sign * cam_dy_m

        return robot["x"], robot["y"]

    def _run_alignment(self, color: str, timeout: float) -> None:
        self._active = True
        self.get_logger().info(f"alignment start: color={color}, timeout={timeout}s")

        color_ranges = self._color_profiles.get(color)
        if color_ranges is None:
            self.get_logger().error(f"unknown color profile: {color}")
            self._publish_status(False, f"unknown_color: {color}")
            self._active = False
            return

        camera_device = self.get_parameter("camera_device").value
        cap = cv2.VideoCapture(camera_device)
        if not cap.isOpened():
            self.get_logger().error(f"cannot open camera device {camera_device}")
            self._publish_status(False, "camera_open_failed")
            self._active = False
            return

        for _ in range(_CAMERA_WARMUP_FRAMES):
            cap.read()

        ring_diameter = self.get_parameter("ring_diameter").value
        tolerance = self.get_parameter("align_tolerance").value
        kp = self.get_parameter("kp").value
        max_speed = self.get_parameter("max_speed").value
        settle_target = self.get_parameter("settle_frames").value
        offset_x_px = self.get_parameter("target_offset_x_px").value
        offset_y_px = self.get_parameter("target_offset_y_px").value
        flip = self.get_parameter("image_flip").value

        settle_count = 0
        miss_count = 0
        interval = 1.0 / _LOOP_HZ
        deadline = time.monotonic() + timeout
        last_dx_m = 0.0
        last_dy_m = 0.0

        try:
            while not self._cancel_event.is_set() and time.monotonic() < deadline:
                ret, frame = cap.read()
                if not ret:
                    miss_count += 1
                    if miss_count > _MAX_MISS_FRAMES:
                        self._stop_robot()
                        self._publish_status(False, "camera_read_failed")
                        return
                    time.sleep(interval)
                    continue

                if flip:
                    frame = cv2.flip(frame, -1)

                h, w = frame.shape[:2]
                target_x = w / 2.0 + offset_x_px
                target_y = h / 2.0 + offset_y_px

                detection = self._detect_ring(frame, color_ranges)

                if detection is None:
                    miss_count += 1
                    settle_count = 0
                    self._stop_robot()
                    if miss_count > _MAX_MISS_FRAMES:
                        self._stop_robot()
                        self._publish_status(False, "ring_not_found")
                        return
                    time.sleep(interval)
                    continue

                miss_count = 0
                cx, cy, radius = detection

                dx_px = cx - target_x
                dy_px = cy - target_y

                m_per_px = ring_diameter / (2.0 * radius) if radius > 0 else 0.0
                robot_dx, robot_dy = self._pixel_to_robot(dx_px, dy_px, m_per_px)
                last_dx_m = robot_dx
                last_dy_m = robot_dy

                dist = math.hypot(robot_dx, robot_dy)

                if dist < tolerance:
                    settle_count += 1
                    self._stop_robot()
                    if settle_count >= settle_target:
                        self._publish_status(True, "aligned", robot_dx, robot_dy)
                        self.get_logger().info(
                            f"alignment success: offset=({robot_dx:.3f}, {robot_dy:.3f})m"
                        )
                        return
                else:
                    settle_count = 0
                    vx = max(-max_speed, min(max_speed, kp * robot_dx))
                    vy = max(-max_speed, min(max_speed, kp * robot_dy))

                    twist = Twist()
                    twist.linear.x = vx
                    twist.linear.y = vy
                    self._pub_cmd_vel.publish(twist)

                time.sleep(interval)

            if self._cancel_event.is_set():
                self._stop_robot()
                self._publish_status(False, "cancelled")
                self.get_logger().info("alignment cancelled")
            else:
                self._stop_robot()
                self._publish_status(False, "timeout",
                                     last_dx_m, last_dy_m)
                self.get_logger().warn(
                    f"alignment timeout: offset=({last_dx_m:.3f}, {last_dy_m:.3f})m"
                )
        finally:
            self._stop_robot()
            cap.release()
            self._active = False


def main(args=None):
    rclpy.init(args=args)
    node = RingAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
