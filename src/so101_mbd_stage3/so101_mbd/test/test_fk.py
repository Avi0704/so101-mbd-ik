import numpy as np
from so101_mbd.kinematics import fk, pose_from_T, T_from_pose, R_from_euler, euler_from_R


def test_rotation_is_orthonormal():
    T = fk([0.3, -0.7, 0.9, 0.2, -1.0])
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0)


def test_pose_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(100):
        a, g = rng.uniform(-3, 3, 2)
        b = rng.uniform(-1.4, 1.4)
        a2, b2, g2 = euler_from_R(R_from_euler(a, b, g))
        assert np.allclose([a, b, g], [a2, b2, g2], atol=1e-9)
