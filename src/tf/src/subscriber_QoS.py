#!/usr/bin/env python3
# sub_best_effort.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class BestEffortSub(Node):
    def __init__(self):
        super().__init__('best_effort_sub')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(String, 'qos_demo', self.cb, qos)
        self.get_logger().info("Best-Effort subscriber started")

    def cb(self, msg):
        self.get_logger().info(f"BEST_EFFORT received: {msg.data}")

def main():
    rclpy.init()
    node = BestEffortSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

