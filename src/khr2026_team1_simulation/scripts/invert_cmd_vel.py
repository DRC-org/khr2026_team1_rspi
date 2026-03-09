#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelInverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_inverter')
        self.sub = self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_inverted', 10)

    def cmd_cb(self, msg: Twist):
        inverted = Twist()
        # Invert linear X and Y to counteract Gazebo MecanumDrive backward interpretation
        inverted.linear.x = -msg.linear.x
        inverted.linear.y = -msg.linear.y
        # preserve angular rotation
        inverted.angular.z = msg.angular.z
        self.pub.publish(inverted)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelInverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
