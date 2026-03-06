import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# slam_toolbox の tf2_ros::MessageFilter が TF 通知コールバックで
# キュー内メッセージを再チェックしない問題への対策。
# スキャンのタイムスタンプを少し過去にずらすことで、
# message filter の初回チェック時に TF が既に利用可能な状態にする。
_DELAY_NS = 100_000_000  # 100ms


class ScanRelayNode(Node):
    def __init__(self):
        super().__init__("scan_relay_node")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, "/scan_delayed", sensor_qos)
        self._sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan, sensor_qos
        )

    def _on_scan(self, msg: LaserScan) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        adjusted = stamp_ns - _DELAY_NS
        msg.header.stamp.sec = int(adjusted // 1_000_000_000)
        msg.header.stamp.nanosec = int(adjusted % 1_000_000_000)
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
