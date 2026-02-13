#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json

class HandBridgeNode(Node):
    def __init__(self):
        super().__init__('hand_bridge_node')
        self.create_subscription(String, '/robot_control', self.control_callback, 10)
        self.lift_pub = self.create_publisher(Float64, '/model/khr2026_robot/joint/lift_joint/cmd_pos', 10)
        self.l_finger_pub = self.create_publisher(Float64, '/model/khr2026_robot/joint/left_finger_joint/cmd_pos', 10)
        self.r_finger_pub = self.create_publisher(Float64, '/model/khr2026_robot/joint/right_finger_joint/cmd_pos', 10)
        self.get_logger().info("Hand Bridge Node Re-initialized")

    def control_callback(self, msg):
        try: data = json.loads(msg.data)
        except: return
        if 'yagura' in data:
            y1 = data['yagura']
            if '1_pos' in y1:
                self.lift_pub.publish(Float64(data=0.4 if y1['1_pos'] == 'up' else 0.0))
            if '1_state' in y1:
                val = 0.5 if y1['1_state'] == 'closed' else 0.0
                self.l_finger_pub.publish(Float64(data=val))
                self.r_finger_pub.publish(Float64(data=-val))

def main():
    rclpy.init(); node = HandBridgeNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
