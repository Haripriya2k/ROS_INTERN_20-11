#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from services.srv import Values # <<-- replace your_package
import math
import sys

class TransformClient(Node):
    def __init__(self):
        super().__init__('transform_client')
        self.client = self.create_client(Values, 'transform_pose')
        self.get_logger().info('Transform client started')

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service "transform_pose" not available. Start the server and retry.')
            raise RuntimeError('Service not available')

    def send_request(self, req):
        future = self.client.call_async(req)
        return future

def get_float(prompt, default=None):
    while True:
        try:
            s = input(prompt)
            if s.strip() == "" and default is not None:
                return default
            return float(s)
        except Exception:
            print("Invalid number. Try again.")

def get_bool(prompt, default=False):
    s = input(prompt + f" [{'Y' if default else 'y'}/{'n' if default else 'N'}]: ").strip().lower()
    if s == "":
        return default
    return s[0] in ('y', 't', '1')

def main(args=None):
    rclpy.init(args=args)
    node = TransformClient()

    try:
        print("\n--- Enter current pose of the object (in WORLD frame) ---")
        px = get_float(" px: ")
        py = get_float(" py: ")
        pz = get_float(" pz: ")

        print("\n Enter current orientation (radians):")
        roll = get_float(" roll (rad): ")
        pitch = get_float(" pitch (rad): ")
        yaw = get_float(" yaw (rad): ")

        print("\n--- Enter ACTION transform to apply ---")
        tx = get_float(" tx: ")
        ty = get_float(" ty: ")
        tz = get_float(" tz: ")

        print("\n Enter action rotation (radians):")
        a_roll = get_float(" action roll (rad): ")
        a_pitch = get_float(" action pitch (rad): ")
        a_yaw = get_float(" action yaw (rad): ")

        action_in_world = get_bool("\nIs the action expressed in WORLD frame? (if no, it's applied in object's local frame)", default=False)

        req = Values.Request()
        req.px = float(px); req.py = float(py); req.pz = float(pz)
        req.roll = float(roll); req.pitch = float(pitch); req.yaw = float(yaw)

        req.tx = float(tx); req.ty = float(ty); req.tz = float(tz)
        req.a_roll = float(a_roll); req.a_pitch = float(a_pitch); req.a_yaw = float(a_yaw)
        req.action_in_world = bool(action_in_world)

        future = node.send_request(req)
        # wait for response
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future.done():
            resp = future.result()
            print("\n--- Result ---")
            print(f" New position: x={resp.new_x:.6f}, y={resp.new_y:.6f}, z={resp.new_z:.6f}")
            print(f" New orientation (radians): roll={resp.new_roll:.6f}, pitch={resp.new_pitch:.6f}, yaw={resp.new_yaw:.6f}")
        else:
            print("No response within timeout.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
