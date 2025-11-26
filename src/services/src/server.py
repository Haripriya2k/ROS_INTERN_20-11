#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from services.srv import Values # <<-- replace your_package

def rpy_to_matrix(roll, pitch, yaw):
    """Return 3x3 rotation matrix for R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)

    # Rx
    # Ry
    # Rz
    # R = Rz * Ry * Rx
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

def mat3_mult(A, B):
    return [
        [
            A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]

def mat3_vec(A, v):
    return [
        A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2],
        A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2],
        A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2],
    ]

def compose_homogeneous(R, t):
    """Return 4x4 homogeneous matrix as a list-of-lists."""
    return [
        [R[0][0], R[0][1], R[0][2], t[0]],
        [R[1][0], R[1][1], R[1][2], t[1]],
        [R[2][0], R[2][1], R[2][2], t[2]],
        [0.0,     0.0,     0.0,     1.0],
    ]

def mat4_mult(A, B):
    # 4x4 multiply
    C = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C

def extract_rpy_from_R(R):
    # R is 3x3 list-of-lists
    r00, r01, r02 = R[0][0], R[0][1], R[0][2]
    r10, r11, r12 = R[1][0], R[1][1], R[1][2]
    r20, r21, r22 = R[2][0], R[2][1], R[2][2]

    # Use stable conversion for R = Rz * Ry * Rx (Tait-Bryan z-y-x)
    if abs(r20) < 0.999999:
        pitch = -math.asin(r20)
        cos_pitch = math.cos(pitch)
        roll = math.atan2(r21 / cos_pitch, r22 / cos_pitch)
        yaw = math.atan2(r10 / cos_pitch, r00 / cos_pitch)
    else:
        # Gimbal lock. Set roll = 0 and compute yaw from elements.
        # If r20 ~= -1 => pitch = +pi/2, if r20 ~= +1 => pitch = -pi/2
        if r20 <= -0.999999:
            pitch = math.pi / 2
        else:
            pitch = -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return roll, pitch, yaw

class TransformServer(Node):
    def __init__(self):
        super().__init__('transform_server')
        self.srv = self.create_service(Values, 'transform_pose', self.handle_transform)
        self.get_logger().info('Transform server ready: service "transform_pose"')

    def handle_transform(self, request, response):
        # Read current pose
        px, py, pz = float(request.px), float(request.py), float(request.pz)
        roll, pitch, yaw = float(request.roll), float(request.pitch), float(request.yaw)

        # Read action
        tx, ty, tz = float(request.tx), float(request.ty), float(request.tz)
        a_roll, a_pitch, a_yaw = float(request.a_roll), float(request.a_pitch), float(request.a_yaw)
        action_in_world = bool(request.action_in_world)

        # Build rotation matrices
        R_point = rpy_to_matrix(roll, pitch, yaw)
        R_action = rpy_to_matrix(a_roll, a_pitch, a_yaw)

        T_point = compose_homogeneous(R_point, [px, py, pz])
        T_action = compose_homogeneous(R_action, [tx, ty, tz])

        if action_in_world:
            # action is a transform in world frame: T_new = T_action * T_point
            T_new = mat4_mult(T_action, T_point)
        else:
            # action is expressed in object's local frame: T_new = T_point * T_action
            T_new = mat4_mult(T_point, T_action)

        # Extract new translation
        new_x = T_new[0][3]
        new_y = T_new[1][3]
        new_z = T_new[2][3]

        # Extract new rotation matrix (upper-left 3x3)
        R_new = [
            [T_new[0][0], T_new[0][1], T_new[0][2]],
            [T_new[1][0], T_new[1][1], T_new[1][2]],
            [T_new[2][0], T_new[2][1], T_new[2][2]],
        ]

        new_roll, new_pitch, new_yaw = extract_rpy_from_R(R_new)

        # Fill response
        response.new_x = float(new_x)
        response.new_y = float(new_y)
        response.new_z = float(new_z)
        response.new_roll = float(new_roll)
        response.new_pitch = float(new_pitch)
        response.new_yaw = float(new_yaw)

        self.get_logger().info(
            f"Computed new pose -> pos=({new_x:.6f}, {new_y:.6f}, {new_z:.6f}) "
            f"rpy=({new_roll:.6f}, {new_pitch:.6f}, {new_yaw:.6f})"
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TransformServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

