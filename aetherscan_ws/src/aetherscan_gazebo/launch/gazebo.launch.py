import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = FindPackageShare('aetherscan_gazebo')
    pkg_description = FindPackageShare('aetherscan_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='office_environment',
        description='World file name (without .sdf extension)',
        choices=['office_environment', 'warehouse', 'apartment']
    )

    world_file = PathJoinSubstitution([
        pkg_gazebo, 'worlds',
        [LaunchConfiguration('world'), '.sdf']
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                pkg_gazebo, 'config', 'bridge_config.yaml'
            ])
        }],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                pkg_description, 'urdf', 'aetherscan_drone.urdf.xacro'
            ]),
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        bridge,
        robot_state_publisher,
    ])
