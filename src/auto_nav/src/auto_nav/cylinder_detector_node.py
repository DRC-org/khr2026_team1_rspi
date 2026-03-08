"""
近距離の円柱（Phi114mm）位置検出ノード

/scan (LaserScan) を購読し、base_link 座標系での円柱中心位置を
cylinder_position (PointStamped) として配信する。

アルゴリズム:
  1. 各スキャン点を laser_frame -> base_link へ座標変換
  2. 指定方位（heading_center_deg +- heading_half_deg）かつ指定距離範囲の点のみ残す
  3. 隣接点間の距離でクラスタリング（スキャン角度順）
  4. 各クラスタに対して Kasa 代数法で最小二乗円フィッティング
  5. 半径が期待値（cylinder_radius +- radius_tolerance）に合うクラスタを採用
  6. 最近傍の候補を cylinder_position として発行

heading_center_deg: 検出方位の中心 (base_link 基準, 0=前, -90=右, 90=左, 180=後)
heading_half_deg  : 検出方位の半開き角 (例: 90 で +-90 度の半円)
"""

import math

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


class CylinderDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("cylinder_detector")

        self.declare_parameter("cylinder_radius", 0.057)   # m (Phi114mm / 2)
        self.declare_parameter("radius_tolerance", 0.025)  # m
        self.declare_parameter("min_range", 0.05)          # m
        self.declare_parameter("max_range", 2.5)           # m
        self.declare_parameter("cluster_gap", 0.10)        # m (クラスタ分割閾値)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("target_frame", "base_link")
        # 検出方位 (base_link 基準, 0=前方, -90=右, 90=左, 180=後方)
        self.declare_parameter("heading_center_deg", 0.0)
        self.declare_parameter("heading_half_deg", 90.0)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan, 10
        )
        self._pub_pos = self.create_publisher(PointStamped, "cylinder_position", 10)
        self._pub_marker = self.create_publisher(MarkerArray, "cylinder_markers", 10)

        self.get_logger().info("CylinderDetector initialized (Phi114mm)")

    def _on_scan(self, msg: LaserScan) -> None:
        r_target = self.get_parameter("cylinder_radius").value
        r_tol = self.get_parameter("radius_tolerance").value
        min_r = self.get_parameter("min_range").value
        max_r = self.get_parameter("max_range").value
        gap = self.get_parameter("cluster_gap").value
        min_pts = int(self.get_parameter("min_cluster_points").value)
        target_frame = self.get_parameter("target_frame").value
        center_rad = math.radians(self.get_parameter("heading_center_deg").value)
        half_rad = math.radians(self.get_parameter("heading_half_deg").value)

        # --- TF 取得 (laser_frame -> base_link) ---
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

        # クォータニオン -> yaw (Z 軸回転のみ想定)
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        # --- 有効スキャン点を base_link 座標へ変換、前方のみ残す ---
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
            diff = abs(math.atan2(math.sin(heading - center_rad), math.cos(heading - center_rad)))
            if diff <= half_rad:
                points_base.append((xb, yb))

        if len(points_base) < min_pts:
            return

        # --- クラスタリング (スキャン角度順の隣接距離ベース) ---
        clusters = _cluster_scan_points(points_base, gap)

        # --- 各クラスタで円フィッティング -> 候補抽出 ---
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

        candidates.sort()
        _, cx, cy, _ = candidates[0]

        pt = PointStamped()
        pt.header.stamp = msg.header.stamp
        pt.header.frame_id = target_frame
        pt.point.x = cx
        pt.point.y = cy
        pt.point.z = 0.0
        self._pub_pos.publish(pt)

        self.get_logger().debug(
            "Cylinder: x=%.3f y=%.3f dist=%.3fm r_fit=%dmm (%d candidates)"
            % (cx, cy, candidates[0][0], int(candidates[0][3] * 1000), len(candidates))
        )

        self._publish_markers(msg.header.stamp, target_frame, candidates)

    def _publish_markers(self, stamp, frame_id, candidates) -> None:
        arr = MarkerArray()
        for i, (_, cx, cy, radius) in enumerate(candidates):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = "cylinder"
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
            m.color.r = 1.0 if i == 0 else 0.5
            m.color.g = 0.5 if i == 0 else 0.5
            m.color.b = 0.0
            m.lifetime.sec = 1
            arr.markers.append(m)
        self._pub_marker.publish(arr)


# ------------------------------------------------------------------
# 純粋関数
# ------------------------------------------------------------------


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

    各点 (x_i, y_i) に対して
        x_i^2 + y_i^2 = 2a*x_i + 2b*y_i + c
    を最小二乗で解き、中心 (a, b) と半径 r を返す。
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
    b = np.array(
        [
            0.5 * float((u**3 + u * v**2).sum()),
            0.5 * float((v**3 + v * u**2).sum()),
        ]
    )

    try:
        uc, vc = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None, None, None

    radius = math.sqrt(uc**2 + vc**2 + (suu + svv) / n)
    return float(uc + mx), float(vc + my), radius


def main(args=None):
    rclpy.init(args=args)
    node = CylinderDetectorNode()
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
