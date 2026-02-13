#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import json

class GzAttachmentNode(Node):
    def __init__(self):
        super().__init__('gz_attachment_node')
        self.objects, self.robot_pos, self.attached_object = {}, None, None
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(String, '/robot_control', self.control_callback, 10)
        self.get_logger().info("GZ Attachment Node Re-initialized")

    def tf_callback(self, msg):
        for t in msg.transforms:
            fid, pos = t.child_frame_id, t.transform.translation
            if fid == 'khr2026_robot': self.robot_pos = pos
            elif fid.startswith('yagura_') or fid.startswith('ring_'): self.objects[fid] = pos

    def control_callback(self, msg):
        try: data = json.loads(msg.data)
        except: return
        y1_state = data.get('yagura', {}).get('1_state')
        if y1_state == 'closed' and not self.attached_object:
            if not self.robot_pos: return
            nearest, min_dist = None, 0.5
            for name, pos in self.objects.items():
                dist = ((pos.x - self.robot_pos.x)**2 + (pos.y - self.robot_pos.y)**2)**0.5
                if dist < min_dist: min_dist, nearest = dist, name
            if nearest:
                self.attached_object = nearest
                self.get_logger().info(f"Gripper: ATTACHED {nearest}")
        elif y1_state == 'open' and self.attached_object:
            self.get_logger().info(f"Gripper: DETACHED {self.attached_object}")
            self.attached_object = None

def main():
    rclpy.init(); node = GzAttachmentNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
