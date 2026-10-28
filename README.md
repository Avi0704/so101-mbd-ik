# mbd_ws — SO-101 multibody kinematics project

## One-time setup (ROS 2 Humble + Gazebo Harmonic)
    sudo apt install ros-humble-ros-gzharmonic ros-humble-ros2-control \
         ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-state-publisher
    # gz_ros2_control for Humble must be built against Harmonic from source:
    cd mbd_ws/src && git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
    cd .. && rosdep install --from-paths src -yir
    export GZ_VERSION=harmonic
    colcon build --symlink-install
    source install/setup.bash

## Run
    ros2 launch so101_gazebo sim.launch.py

## Check
    ros2 control list_controllers          # all three should be "active"
    ros2 topic echo /joint_states --once
    ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.5, 0.5, 0.0, 0.0]}"
