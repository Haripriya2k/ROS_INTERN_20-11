#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from wheels.action import CountUntil

def read_float(prompt, default=0.0):
    try:
        s = input(prompt)
        if s.strip() == "":
            return float(default)
        return float(s)
    except Exception:
        print("invalid input, using", default)
        return float(default)

def read_int(prompt, default=5):
    try:
        s = input(prompt)
        if s.strip() == "":
            return int(default)
        return int(s)
    except Exception:
        print("invalid input, using", default)
        return int(default)

def read_bool(prompt, default=False):
    s = input(prompt + f" [{'Y' if default else 'y'}/{'n' if default else 'N'}]: ").strip().lower()
    if s == "":
        return default
    return s[0] in ('y', 't', '1')

class TransformActionClient(Node):
    def __init__(self):
        super().__init__('count_until_action_client')  # keep name consistent
        self._action_client = ActionClient(self, CountUntil, 'count')
        self.get_logger().info('Transform (CountUntil) Action Client started')

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        p = fb.homogeneous_point
        self.get_logger().info(f"Feedback: progress={fb.progress:.3f} point=({p.x:.4f},{p.y:.4f},{p.z:.4f})")

    def send_goal(self, goal_msg):
        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

def main(args=None):
    rclpy.init(args=args)
    node = TransformActionClient()

    # wait for server
    if not node._action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Action server not available. Start the server and retry.')
        return

    try:
        print("\n--- Current object pose (WORLD frame) ---")
        px = read_float(" px: ")
        py = read_float(" py: ")
        pz = read_float(" pz: ")

        print("\n--- Current orientation (radians) ---")
        roll = read_float(" roll (rad): ")
        pitch = read_float(" pitch (rad): ")
        yaw = read_float(" yaw (rad): ")

        print("\n--- ACTION transform to apply (translation + rotation in radians) ---")
        tx = read_float(" tx: ")
        ty = read_float(" ty: ")
        tz = read_float(" tz: ")

        a_roll = read_float(" action roll (rad): ")
        a_pitch = read_float(" action pitch (rad): ")
        a_yaw = read_float(" action yaw (rad): ")

        action_in_world = read_bool("\nIs the action expressed in WORLD frame? (if no, it's applied in object's local frame)", default=False)

        steps = read_int("\nHow many feedback steps to receive? (e.g. 5) [default 5]: ", default=5)

        goal = CountUntil.Goal()
        # set exact field names as per action file
        goal.px = float(px); goal.py = float(py); goal.pz = float(pz)
        goal.roll = float(roll); goal.pitch = float(pitch); goal.yaw = float(yaw)
        goal.tx = float(tx); goal.ty = float(ty); goal.tz = float(tz)
        goal.a_roll = float(a_roll); goal.a_pitch = float(a_pitch); goal.a_yaw = float(a_yaw)
        goal.action_in_world = bool(action_in_world)
        goal.steps = int(steps)

        fut = node.send_goal(goal)
        rclpy.spin_until_future_complete(node, fut)
        goal_handle = fut.result()
        if not goal_handle.accepted:
            node.get_logger().info('Goal rejected by server')
            return

        node.get_logger().info('Goal accepted — waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        res = result_future.result().result

        print("\n--- Final result (from server) ---")
        print(f" New position: x={res.new_x:.6f}, y={res.new_y:.6f}, z={res.new_z:.6f}")
        print(f" New orientation (radians): roll={res.new_roll:.6f}, pitch={res.new_pitch:.6f}, yaw={res.new_yaw:.6f}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
