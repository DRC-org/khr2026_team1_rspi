import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.subscription = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.get_logger().info('Object Detector Node started.')

    def image_callback(self, msg):
        # Simplified for compatibility (no cv_bridge/cv2 to avoid NumPy 2.x crash)
        # In real scenario, would use cv_bridge here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
