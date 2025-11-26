#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Point
from wheels.action import CountUntil

# ---------------- math helpers ----------------
def rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr

    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr

    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    return [
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ]

def compose_homogeneous(R, t):
    return [
        [R[0][0], R[0][1], R[0][2], t[0]],
        [R[1][0], R[1][1], R[1][2], t[1]],
        [R[2][0], R[2][1], R[2][2], t[2]],
        [0.0,     0.0,     0.0,     1.0],
    ]

def mat4_mult(A, B):
    C = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C

def extract_translation(T):
    return T[0][3], T[1][3], T[2][3]

def extract_rpy_from_R(R):
    r00, r01, r02 = R[0][0], R[0][1], R[0][2]
    r10, r11, r12 = R[1][0], R[1][1], R[1][2]
    r20, r21, r22 = R[2][0], R[2][1], R[2][2]
    if abs(r20) < 0.999999:
        pitch = -math.asin(r20)
        cos_pitch = math.cos(pitch)
        roll = math.atan2(r21 / cos_pitch, r22 / cos_pitch)
        yaw = math.atan2(r10 / cos_pitch, r00 / cos_pitch)
    else:
        if r20 <= -0.999999:
            pitch = math.pi / 2.0
        else:
            pitch = -math.pi / 2.0
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return roll, pitch, yaw

# ---------------- Action Server ----------------
class TransformActionServer(Node):
    def __init__(self):
        super().__init__('count_until_action_server')  # keep original node name if you like
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count',                 # action name as in your package
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('Transform (CountUntil) Action Server started.')

    def goal_callback(self, goal_request):
        # Validate steps present and positive
        if not hasattr(goal_request, 'steps'):
            self.get_logger().warn("Goal missing 'steps' field -> rejecting")
            return GoalResponse.REJECT
        try:
            steps = int(goal_request.steps)
        except Exception:
            self.get_logger().warn("Invalid steps -> rejecting")
            return GoalResponse.REJECT
        if steps <= 0:
            self.get_logger().warn("Non-positive steps -> rejecting")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted transform goal: px={goal_request.px:.3f},py={goal_request.py:.3f},pz={goal_request.pz:.3f} "
            f"roll={goal_request.roll:.3f},pitch={goal_request.pitch:.3f},yaw={goal_request.yaw:.3f} "
            f" tx=({goal_request.tx:.3f},{goal_request.ty:.3f},{goal_request.tz:.3f}) "
            f"a_rpy=({goal_request.a_roll:.3f},{goal_request.a_pitch:.3f},{goal_request.a_yaw:.3f}) "
            f"action_in_world={bool(goal_request.action_in_world)} steps={steps}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        req = goal_handle.request

        # Build original transform (object pose)
        R_point = rpy_to_matrix(req.roll, req.pitch, req.yaw)
        T_point = compose_homogeneous(R_point, [float(req.px), float(req.py), float(req.pz)])

        # Action transform (to apply)
        R_action_full = rpy_to_matrix(req.a_roll, req.a_pitch, req.a_yaw)
        T_action_full = compose_homogeneous(R_action_full, [float(req.tx), float(req.ty), float(req.tz)])

        steps = int(req.steps)
        feedback = CountUntil.Feedback()

        # For feedback we interpolate the action by scaling the action's rpy and translation linearly:
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client')
                return CountUntil.Result()

            frac = float(i + 1) / float(steps)  # 0..1
            # scale rotation and translation of the action transform for intermediate step
            s_roll = req.a_roll * frac
            s_pitch = req.a_pitch * frac
            s_yaw = req.a_yaw * frac
            s_tx = float(req.tx) * frac
            s_ty = float(req.ty) * frac
            s_tz = float(req.tz) * frac

            R_action = rpy_to_matrix(s_roll, s_pitch, s_yaw)
            T_action = compose_homogeneous(R_action, [s_tx, s_ty, s_tz])

            if bool(req.action_in_world):
                # T_new_step = T_action_scaled * T_point
                T_new_step = mat4_mult(T_action, T_point)
            else:
                # T_new_step = T_point * T_action_scaled
                T_new_step = mat4_mult(T_point, T_action)

            nx, ny, nz = extract_translation(T_new_step)

            p = Point()
            p.x = float(nx)
            p.y = float(ny)
            p.z = float(nz)
            feedback.homogeneous_point = p
            feedback.progress = float(frac)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Feedback {i+1}/{steps}: point=({p.x:.4f},{p.y:.4f},{p.z:.4f}) progress={frac:.3f}")

            # simulate computation time (tune or remove)
            time.sleep(0.08)

        # Compute final transform using full action
        if bool(req.action_in_world):
            T_new = mat4_mult(T_action_full, T_point)
        else:
            T_new = mat4_mult(T_point, T_action_full)

        new_x, new_y, new_z = extract_translation(T_new)
        # extract rotation matrix from T_new
        R_new = [
            [T_new[0][0], T_new[0][1], T_new[0][2]],
            [T_new[1][0], T_new[1][1], T_new[1][2]],
            [T_new[2][0], T_new[2][1], T_new[2][2]],
        ]
        new_roll, new_pitch, new_yaw = extract_rpy_from_R(R_new)

        # Prepare result
        result = CountUntil.Result()
        # set result fields (these names match the action file)
        result.new_x = float(new_x)
        result.new_y = float(new_y)
        result.new_z = float(new_z)
        result.new_roll = float(new_roll)
        result.new_pitch = float(new_pitch)
        result.new_yaw = float(new_yaw)

        goal_handle.succeed()
        self.get_logger().info(f"Goal succeeded. final=({new_x:.6f},{new_y:.6f},{new_z:.6f})")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TransformActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
