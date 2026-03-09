#!/usr/bin/env python3

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BtAutoSequenceSim(Node):
    """Simulate Bluetooth auto-sequence commands in simulation.

    This node publishes the same JSON commands that the physical controller
    would send via Bluetooth:
      1) set_court (red)
      2) nav_mode: auto
      3) start_auto (from_index=0)
    """

    def __init__(self) -> None:
        super().__init__("bt_auto_sequence_sim")
        self._pub = self.create_publisher(String, "bluetooth_rx", 10)

        # Run sequence in background so node can spin normally
        t = threading.Thread(target=self._run_sequence, daemon=True)
        t.start()

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)
        self.get_logger().info(f"Published BT sim command: {msg.data}")

    def _run_sequence(self) -> None:
        # Wait for Nav2 / routing_node / bridges to come up
        time.sleep(10.0)

        # 1) 赤コートを選択
        self._publish({"type": "set_court", "court": "red"})
        time.sleep(1.0)

        # 2) 自動モードへ
        self._publish({"type": "nav_mode", "mode": "auto"})
        time.sleep(1.0)

        # 3) 自動シーケンス開始（waypoints.yaml の auto_sequence 先頭から）
        self._publish({"type": "start_auto", "from_index": 0})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BtAutoSequenceSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

