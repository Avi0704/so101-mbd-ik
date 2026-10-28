"""Print the FK pose for given joint angles, to compare with Gazebo's TF.

    ros2 run so101_mbd fk_check 0 -0.5 0.5 0 0
    ros2 run tf2_ros tf2_echo base_link gripper_frame_link     # while the arm is at those angles
"""
import sys
import numpy as np
from so101_mbd.kinematics import fk, pose_from_T


def main():
    q = np.array([float(v) for v in sys.argv[1:6]]) if len(sys.argv) >= 6 else np.zeros(5)
    T = fk(q)
    np.set_printoptions(precision=5, suppress=True)
    print('q =', q)
    print('T =\n', T)
    print('pose [x y z alpha beta gamma] =', pose_from_T(T))


if __name__ == '__main__':
    main()
