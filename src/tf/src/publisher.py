#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf.msg import Tf

class SimplePublisher(Node):
    def __init__(self, values):
        super().__init__("simple_transform_publisher")
        self.pub = self.create_publisher(Tf, "my_transform", 10)
        self.values = values
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Tf()
        msg.tx = self.values["tx"]
        msg.ty = self.values["ty"]
        msg.tz = self.values["tz"]
        msg.roll  = math.radians(self.values["roll_deg"])
        msg.pitch = math.radians(self.values["pitch_deg"])
        msg.yaw   = math.radians(self.values["yaw_deg"])

        self.pub.publish(msg)
        self.get_logger().info(
            f"Published: TX={msg.tx}, TY={msg.ty}, TZ={msg.tz}, "
            f"Roll={self.values['roll_deg']}°, Pitch={self.values['pitch_deg']}°, "
            f"Yaw={self.values['yaw_deg']}°"
        )


def main():
    tx = float(input("Enter TX: "))
    ty = float(input("Enter TY: "))
    tz = float(input("Enter TZ: "))
    roll_deg  = float(input("Enter Roll (deg): "))
    pitch_deg = float(input("Enter Pitch (deg): "))
    yaw_deg   = float(input("Enter Yaw (deg): "))

    values = {
        "tx": tx,
        "ty": ty,
        "tz": tz,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg
    }

    rclpy.init()
    node = SimplePublisher(values)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

