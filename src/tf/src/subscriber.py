#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf.msg import Tf

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_transform_subscriber")
        self.sub = self.create_subscription(
            Tf,
            "my_transform",
            self.callback,
            10
        )

    def callback(self, msg: Tf):
        roll_deg  = math.degrees(msg.roll)
        pitch_deg = math.degrees(msg.pitch)
        yaw_deg   = math.degrees(msg.yaw)

        self.get_logger().info(
            f"\nReceived Tf:"
            f"\n  TX = {msg.tx}"
            f"\n  TY = {msg.ty}"
            f"\n  TZ = {msg.tz}"
            f"\n  Roll  = {msg.roll:.3f} rad ({roll_deg:.2f}°)"
            f"\n  Pitch = {msg.pitch:.3f} rad ({pitch_deg:.2f}°)"
            f"\n  Yaw   = {msg.yaw:.3f} rad ({yaw_deg:.2f}°)\n"
        )


def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

