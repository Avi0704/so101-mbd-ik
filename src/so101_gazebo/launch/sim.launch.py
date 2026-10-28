import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('so101_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'so101.urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)

    # lets Gazebo resolve package://so101_description/meshes/...
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(desc_share, '..'))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'so101'],
                 output='screen')

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'])

    jsb, arm, grip = spawner('joint_state_broadcaster'), spawner('arm_controller'), spawner('gripper_controller')

    chain = [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, grip])),
    ]
    return LaunchDescription([resource_path, gz, clock, rsp, spawn, *chain])
