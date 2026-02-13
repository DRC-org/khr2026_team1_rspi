#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, String
import json
from robot_control.scoring import ScoringManager

class ScoringNode(Node):
    def __init__(self):
        super().__init__('scoring_node')
        self.sm = ScoringManager(is_auto=True)
        self.score_pub = self.create_publisher(Int32, '/score', 10)
        self.score_detail_pub = self.create_publisher(String, '/score_detail', 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_timer(1.0, self.publish_score)
        self.zones = {1: {"x": 3.15, "y": 2.0}, 2: {"x": 3.15, "y": 3.2}, 3: {"x": 3.15, "y": 4.4}}
        self.get_logger().info("Scoring Node Re-initialized")

    def tf_callback(self, msg):
        zone_states = {zid: {'y': 0, 'r_in': 0, 'r_floor': 0} for zid in self.zones}
        honmaru_rings = 0
        for t in msg.transforms:
            fid, pos = t.child_frame_id, t.transform.translation
            if fid == 'khr2026_robot' and 0.5 < pos.y < 1.5: self.sm.report_yagura_zone_entry()
            if fid.startswith('yagura_'):
                for zid, zpos in self.zones.items():
                    if ((pos.x - zpos['x'])**2 + (pos.y - zpos['y'])**2)**0.5 < 0.3: zone_states[zid]['y'] += 1
            if fid.startswith('ring_'):
                if ((pos.x - 3.25)**2 + (pos.y - 1.40)**2)**0.5 < 0.15 and pos.z > 0.3: honmaru_rings += 1
                else:
                    for zid, zpos in self.zones.items():
                        if ((pos.x - zpos['x'])**2 + (pos.y - zpos['y'])**2)**0.5 < 0.3:
                            if zone_states[zid]['y'] > 0: zone_states[zid]['r_in'] += 1
                            else: zone_states[zid]['r_floor'] += 1
        self.sm.update_honmaru_state(honmaru_rings)
        for zid, s in zone_states.items():
            if s['y'] > 0 or s['r_floor'] > 0: self.sm.update_zone_state(zid, s['y'], s['r_in'], s['r_floor'])

    def publish_score(self):
        self.score_pub.publish(Int32(data=self.sm.get_total_score()))
        detail = {"total_score": self.sm.get_total_score(), "ote": self.sm.is_ote(), "v_goal": self.sm.is_v_goal()}
        self.score_detail_pub.publish(String(data=json.dumps(detail)))

def main():
    rclpy.init(); node = ScoringNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
