# so101-mbd-ik

Position and velocity inverse kinematics for the SO-101 arm, built with matrix methods (rotation/transformation matrices and the Jacobian) for a multibody dynamics course project. Simulated in Gazebo.

## Packages (in `src/`)
- `so101_description`: URDF/xacro and meshes
- `so101_gazebo`: Gazebo launch and controller config
- `so101_mbd`: forward kinematics, Jacobian, and verification scripts (`fk_check`, `jac_check`)

## Dependencies
Clone these into `src/`:

```bash
git clone https://github.com/gazebosim/ros_gz.git
git clone https://github.com/ros-controls/gz_ros2_control.git
```

## Build
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run
```bash
ros2 launch so101_gazebo sim.launch.py
```
