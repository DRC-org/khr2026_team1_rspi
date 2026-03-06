import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# slam_toolbox の tf2_ros::MessageFilter が TF 通知コールバックで
# キュー内メッセージを再チェックしない問題への対策。
# スキャンを実時間で遅延させてから配信することで、
# MessageFilter の初回チェック時に TF が確実に利用可能な状態にする。
# タイムスタンプは変更しない（スキャンデータとポーズの整合性を保つ）。
_DELAY_SEC = 0.15


class ScanRelayNode(Node):
    def __init__(self):
        super().__init__("scan_relay_node")

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, "/scan_delayed", best_effort_qos)
        self._sub = self.create_subscription(
            LaserScan, "/scan_filtered", self._on_scan, reliable_qos
        )
        self._buffer: deque[tuple[float, LaserScan]] = deque()
        self._timer = self.create_timer(0.01, self._flush)

    def _on_scan(self, msg: LaserScan) -> None:
        self._buffer.append((time.monotonic(), msg))

    def _flush(self) -> None:
        now = time.monotonic()
        while self._buffer and now - self._buffer[0][0] >= _DELAY_SEC:
            _, msg = self._buffer.popleft()
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelayNode()
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
