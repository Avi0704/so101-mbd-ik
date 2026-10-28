"""Print J(q), its singular values, and the tool velocity for a joint-speed vector.

    ros2 run so101_mbd jac_check 0.5 -0.5 0.5 0.3 0.4
"""
import sys
import numpy as np
from so101_mbd.jacobian import jacobian, singular_values


def main():
    q = np.array([float(v) for v in sys.argv[1:6]]) if len(sys.argv) >= 6 else np.array([0, -0.5, 0.5, 0, 0.0])
    np.set_printoptions(precision=4, suppress=True)
    print('q =', q)
    print('J (rows: wx wy wz vx vy vz) =\n', jacobian(q))
    print('singular values =', singular_values(q))


if __name__ == '__main__':
    main()
