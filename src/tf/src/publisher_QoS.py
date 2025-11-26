#!/usr/bin/env python3
# publisher_skip_toggle.py
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class SkipPublisher(Node):
    def __init__(self, reliability, skip_every):
        super().__init__('skip_publisher')
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability
        )
        self.pub = self.create_publisher(String, 'qos_demo', qos)
        self.count = 0
        self.skip_every = skip_every
        self.timer = self.create_timer(1.0 / 5.0, self.on_timer)  # 5 Hz
        self.get_logger().info(f"Publisher started (reliability={reliability}). skip_every={skip_every}")

    def on_timer(self):
        self.count += 1
        if self.skip_every > 0 and (self.count % self.skip_every == 0):
            # simulate a skip/gap (no publish)
            self.get_logger().warn(f"Intentionally SKIPPING publish #{self.count}")
            return
        msg = String()
        msg.data = f"msg #{self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reliable', action='store_true', help='Use RELIABLE reliability')
    parser.add_argument('--skip', type=int, default=4, help='Skip every Nth publish; 0 disables skipping')
    args = parser.parse_args()

    rclpy.init()
    reliability = QoSReliabilityPolicy.RELIABLE if args.reliable else QoSReliabilityPolicy.BEST_EFFORT
    node = SkipPublisher(reliability=reliability, skip_every=args.skip)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

