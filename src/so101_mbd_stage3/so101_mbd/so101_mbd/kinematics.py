"""
Forward kinematics of the SO-101, written with plain 4x4 transformation matrices.

Idea (this is the whole of FK):
    Every joint i is "a fixed offset from the previous link" followed by
    "a rotation about the joint's own z-axis by q_i":

        A_i(q_i) = T_fixed_i . Rz(q_i)

    and the end-effector pose in the base frame is the product

        T(q) = A_1(q_1) A_2(q_2) A_3(q_3) A_4(q_4) A_5(q_5) . T_tool

T_fixed_i is a 4x4 matrix  [ R  p ]  built from the <origin xyz rpy> of the URDF:
                           [ 0  1 ]
p = xyz, and R = Rz(yaw) Ry(pitch) Rx(roll)   (URDF's rpy convention).
"""
import numpy as np

# ----------------------------------------------------------------------------
# Elementary rotations
# ----------------------------------------------------------------------------
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def make_T(R, p):
    """Assemble a 4x4 homogeneous transform from a 3x3 rotation and a 3-vector."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def rotz_T(q):
    """4x4 pure rotation about z by q (the moving part of a revolute joint)."""
    return make_T(rot_z(q), [0.0, 0.0, 0.0])


def fixed_T(xyz, rpy):
    """4x4 from a URDF <origin xyz rpy>.  R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    return make_T(R, xyz)


# ----------------------------------------------------------------------------
# Robot data, copied from so101_new_calib.urdf (parent -> child, axis = local z)
# ----------------------------------------------------------------------------
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# (xyz, rpy) of each joint frame relative to the previous link frame
_FIXED = [
    ((0.0388353, -8.97657e-09, 0.0624),      (3.14159, 4.18253e-17, -3.14159)),   # shoulder_pan
    ((-0.0303992, -0.0182778, -0.0542),      (-1.5708, -1.5708, 0.0)),            # shoulder_lift
    ((-0.11257, -0.028, 1.73763e-16),        (-3.63608e-16, 8.74301e-16, 1.5708)),# elbow_flex
    ((-0.1349, 0.0052, 3.62355e-17),         (4.02456e-15, 8.67362e-16, -1.5708)),# wrist_flex
    ((5.55112e-17, -0.0611, 0.0181),         (1.5708, 0.0486795, 3.14159)),       # wrist_roll
]
# gripper_link -> gripper_frame_link (the tool centre point we want to place)
_TOOL = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))

T_FIXED = [fixed_T(xyz, rpy) for xyz, rpy in _FIXED]
# The URDF tool frame has its x-axis pointing straight down at home, which sits next to the
# Z-Y-X Euler singularity (beta = +-90 deg). We therefore add a fixed +90 deg rotation about the
# tool's own z-axis (the approach direction stays the same). Now x is horizontal at home and
# beta ~ 0 when the gripper points straight down, so typed targets stay well-behaved.
TOOL_ALIGN = make_T(rot_z(np.pi / 2), [0.0, 0.0, 0.0])
T_TOOL = fixed_T(*_TOOL) @ TOOL_ALIGN

# Joint limits [rad] from the URDF
Q_MIN = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])
Q_MAX = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121])


# ----------------------------------------------------------------------------
# Forward kinematics
# ----------------------------------------------------------------------------
def joint_frames(q):
    """
    Returns (frames_before, frames_after, T_ee).

    frames_before[i] = pose of joint i's frame (after its fixed offset, BEFORE rotating by q_i).
                       Its z-axis is the joint axis in the base frame - needed by the Jacobian.
    frames_after[i]  = same frame after the rotation q_i (this is the link frame).
    T_ee             = pose of the tool frame in the base frame.
    """
    T = np.eye(4)
    before, after = [], []
    for Tf, qi in zip(T_FIXED, q):
        T = T @ Tf
        before.append(T.copy())
        T = T @ rotz_T(qi)
        after.append(T.copy())
    return before, after, T @ T_TOOL


def fk(q):
    """4x4 end-effector pose in the base frame for joint vector q (5 values, rad)."""
    return joint_frames(np.asarray(q, dtype=float))[2]


# ----------------------------------------------------------------------------
# Pose <-> (x, y, z, alpha, beta, gamma).   Convention: Z-Y-X
#   R = Rz(alpha) Ry(beta) Rx(gamma)      alpha = yaw, beta = pitch, gamma = roll
# Change here if your course uses another convention.
# ----------------------------------------------------------------------------
def R_from_euler(alpha, beta, gamma):
    return rot_z(alpha) @ rot_y(beta) @ rot_x(gamma)


def euler_from_R(R):
    """Inverse of R_from_euler. Returns (alpha, beta, gamma)."""
    sb = -R[2, 0]
    beta = np.arcsin(np.clip(sb, -1.0, 1.0))
    if abs(sb) < 1.0 - 1e-9:
        alpha = np.arctan2(R[1, 0], R[0, 0])
        gamma = np.arctan2(R[2, 1], R[2, 2])
    else:  # gimbal lock: alpha and gamma are not separable, pick gamma = 0
        gamma = 0.0
        alpha = np.arctan2(-R[0, 1], R[1, 1])
    return alpha, beta, gamma


def T_from_pose(x, y, z, alpha, beta, gamma):
    return make_T(R_from_euler(alpha, beta, gamma), [x, y, z])


def pose_from_T(T):
    a, b, g = euler_from_R(T[:3, :3])
    return np.array([T[0, 3], T[1, 3], T[2, 3], a, b, g])
