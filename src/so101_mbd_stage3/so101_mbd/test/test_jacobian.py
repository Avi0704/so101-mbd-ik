import numpy as np
from so101_mbd.kinematics import fk, Q_MIN, Q_MAX
from so101_mbd.jacobian import jacobian


def numeric_jacobian(q, h=1e-6):
    """Differentiate the FK numerically, one joint at a time (central difference)."""
    q = np.asarray(q, float)
    T0 = fk(q)
    J = np.zeros((6, 5))
    for i in range(5):
        dq = np.zeros(5); dq[i] = h
        Tp, Tm = fk(q + dq), fk(q - dq)
        J[3:, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)               # d(position)/dq_i
        dR = (Tp[:3, :3] - Tm[:3, :3]) / (2 * h)                    # dR/dq_i
        W = dR @ T0[:3, :3].T                                       # skew-symmetric [w]
        J[:3, i] = [W[2, 1], W[0, 2], W[1, 0]]                      # "vee": extract w
    return J


def test_jacobian_matches_numeric_derivative():
    rng = np.random.default_rng(2)
    for _ in range(200):
        q = rng.uniform(Q_MIN, Q_MAX)
        assert np.allclose(jacobian(q), numeric_jacobian(q), atol=1e-6)
