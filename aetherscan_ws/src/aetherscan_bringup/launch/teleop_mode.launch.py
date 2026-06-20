"""Launch teleop mode - simulation with manual control only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = FindPackageShare('aetherscan_gazebo')
    pkg_control = FindPackageShare('aetherscan_control')

    world_arg = DeclareLaunchArgument(
        'world', default_value='office_environment',
        description='Gazebo world to load'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    control_params = PathJoinSubstitution([pkg_control, 'config', 'control_params.yaml'])

    flight_controller = Node(
        package='aetherscan_control',
        executable='flight_controller',
        name='flight_controller',
        parameters=[control_params, {'use_sim_time': True}],
        output='screen'
    )

    obstacle_avoidance = Node(
        package='aetherscan_navigation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        flight_controller,
        obstacle_avoidance,
    ])
