"""
Geometric Jacobian of the SO-101 (5 joints -> 6D tool velocity).

Velocity kinematics:      [ w ]            [ w ]   angular velocity of the tool   (in the base frame)
                          [ v ]  = J(q) qd [ v ]   linear velocity of the tool point (in the base frame)

J is 6 x 5.  Column i is the effect of joint i alone (all other joints held still):

    revolute joint i with axis z_i (unit vector, base frame) through point p_i:

        angular part:   w_i = z_i                       (the tool turns about z_i at rate qd_i)
        linear part:    v_i = z_i x (p_ee - p_i)        (the tool point swings around the axis: v = w x r)

    column_i = [ z_i ; z_i x (p_ee - p_i) ]

z_i and p_i come straight from the FK: they are the z-axis and origin of joint i's frame
(joint_frames()[0][i]).  Angular part on top, linear part below (same order as a twist in
Modern Robotics).
"""
import numpy as np
from so101_mbd.kinematics import joint_frames


def jacobian(q):
    q = np.asarray(q, dtype=float)
    before, _, T_ee = joint_frames(q)
    p_ee = T_ee[:3, 3]
    J = np.zeros((6, len(q)))
    for i, F in enumerate(before):
        z = F[:3, 2]          # joint axis in base frame
        p = F[:3, 3]          # a point on that axis
        J[:3, i] = z
        J[3:, i] = np.cross(z, p_ee - p)
    return J


def tool_velocity(q, qdot):
    """Forward velocity kinematics: joint speeds -> [wx wy wz vx vy vz] of the tool."""
    return jacobian(q) @ np.asarray(qdot, dtype=float)


def singular_values(q):
    """Small last value => close to a singularity (arm loses ability to move in some direction)."""
    return np.linalg.svd(jacobian(q), compute_uv=False)
