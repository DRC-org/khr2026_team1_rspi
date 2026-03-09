"""
yagura_position_node — 櫓（Phi114mm）2D座標検出（最大3本）

/scan を購読し、全周から Phi114mm 円柱を最大 3 本検出し、
距離が近い順に base_link 座標系の PointStamped で配信する。

トピック:
  yagura_position_0  最も近い櫓
  yagura_position_1  2番目に近い櫓
  yagura_position_2  3番目に近い櫓
  yagura_markers     RViz 可視化

パラメータ:
  cylinder_radius   期待半径 [m] (デフォルト 0.057 = Phi114mm/2)
  radius_tolerance  半径の許容誤差 [m] (デフォルト 0.015 = ±15mm)
  min_range         最小検出距離 [m] (デフォルト 0.05)
  max_range         最大検出距離 [m] (デフォルト 2.5)
  max_yagura        最大検出数 (デフォルト 3)
"""

import math

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

_MAX_YAGURA = 3

# 前方検出範囲: base_link x+ 方向を中心に ±60 度
_FRONT_CENTER_RAD = 0.0
_FRONT_HALF_RAD = math.radians(60.0)


class YaguraPositionNode(Node):
    def __init__(self) -> None:
        super().__init__("yagura_position")

        self.declare_parameter("cylinder_radius", 0.057)   # m (Phi114mm / 2)
        self.declare_parameter("radius_tolerance", 0.015)  # m
        self.declare_parameter("min_range", 0.05)          # m
        self.declare_parameter("max_range", 2.5)           # m
        self.declare_parameter("cluster_gap", 0.10)        # m
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("max_yagura", _MAX_YAGURA)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )
        self._pub_pos = [
            self.create_publisher(PointStamped, f"yagura_position_{i}", 10)
            for i in range(_MAX_YAGURA)
        ]
        self._pub_marker = self.create_publisher(MarkerArray, "yagura_markers", 10)

        self.get_logger().info("YaguraPosition initialized (Phi114mm, all directions, max 3)")

    def _on_scan(self, msg: LaserScan) -> None:
        r_target = self.get_parameter("cylinder_radius").value
        r_tol = self.get_parameter("radius_tolerance").value
        min_r = self.get_parameter("min_range").value
        max_r = self.get_parameter("max_range").value
        gap = self.get_parameter("cluster_gap").value
        min_pts = int(self.get_parameter("min_cluster_points").value)
        target_frame = self.get_parameter("target_frame").value

        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as e:
            self.get_logger().warn(
                "TF lookup failed: %s" % e, throttle_duration_sec=5.0
            )
            return

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        max_yagura = int(self.get_parameter("max_yagura").value)

        # 前方 ±60° の点を base_link 座標に変換
        points_base = []
        for i, r in enumerate(msg.ranges):
            if not (min_r <= r <= max_r) or not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            xl = r * math.cos(angle)
            yl = r * math.sin(angle)
            xb = cos_y * xl - sin_y * yl + tx
            yb = sin_y * xl + cos_y * yl + ty
            heading = math.atan2(yb, xb)
            diff = abs(math.atan2(
                math.sin(heading - _FRONT_CENTER_RAD),
                math.cos(heading - _FRONT_CENTER_RAD),
            ))
            if diff <= _FRONT_HALF_RAD:
                points_base.append((xb, yb))

        if len(points_base) < min_pts:
            return

        clusters = _cluster_scan_points(points_base, gap)

        # 各クラスタで円フィッティング → 距離順にソート
        candidates = []
        for cl in clusters:
            if len(cl) < min_pts:
                continue
            cx, cy, radius = _fit_circle(cl)
            if cx is None:
                continue
            if abs(radius - r_target) <= r_tol:
                dist = math.hypot(cx, cy)
                candidates.append((dist, cx, cy, radius))

        if not candidates:
            return

        candidates.sort()  # 距離近い順

        # 最大 max_yagura 本を publish
        for idx, (dist, cx, cy, radius) in enumerate(candidates[:max_yagura]):
            pt = PointStamped()
            pt.header.stamp = msg.header.stamp
            pt.header.frame_id = target_frame
            pt.point.x = cx
            pt.point.y = cy
            pt.point.z = 0.0
            self._pub_pos[idx].publish(pt)

        self.get_logger().debug(
            "Yagura: %d detected (of %d candidates)"
            % (min(len(candidates), max_yagura), len(candidates))
        )

        self._publish_markers(msg.header.stamp, target_frame, candidates[:max_yagura])

    def _publish_markers(self, stamp, frame_id, candidates) -> None:
        colors = [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        arr = MarkerArray()
        for i, (_, cx, cy, radius) in enumerate(candidates):
            r, g, b = colors[i] if i < len(colors) else (0.5, 0.5, 0.5)
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = "yagura"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = radius * 2.0
            m.scale.y = radius * 2.0
            m.scale.z = 0.4
            m.color.a = 0.7
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.lifetime.sec = 1
            arr.markers.append(m)
        self._pub_marker.publish(arr)


def _cluster_scan_points(pts, gap):
    """スキャン角度順に並んだ点列を Euclidean ギャップでクラスタ分割する。"""
    clusters = []
    current = [pts[0]]
    for prev, curr in zip(pts, pts[1:]):
        d = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        if d < gap:
            current.append(curr)
        else:
            clusters.append(current)
            current = [curr]
    clusters.append(current)
    return clusters


def _fit_circle(pts):
    """
    Kasa 代数的最小二乗法による円フィッティング。
    点数が 3 未満または行列が特異の場合は (None, None, None)。
    """
    arr = np.asarray(pts, dtype=np.float64)
    n = len(arr)
    if n < 3:
        return None, None, None

    mx = arr[:, 0].mean()
    my = arr[:, 1].mean()
    u = arr[:, 0] - mx
    v = arr[:, 1] - my

    suu = float((u * u).sum())
    svv = float((v * v).sum())
    suv = float((u * v).sum())

    A = np.array([[suu, suv], [suv, svv]])
    b = np.array([
        0.5 * float((u**3 + u * v**2).sum()),
        0.5 * float((v**3 + v * u**2).sum()),
    ])

    try:
        uc, vc = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None, None, None

    radius = math.sqrt(uc**2 + vc**2 + (suu + svv) / n)
    return float(uc + mx), float(vc + my), radius


def main(args=None):
    rclpy.init(args=args)
    node = YaguraPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
