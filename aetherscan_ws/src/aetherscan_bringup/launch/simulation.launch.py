"""Master launch file for the full AetherScan simulation system.

Brings up: Gazebo, drone, bridges, SLAM, navigation, control, mission, rosbridge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = FindPackageShare('aetherscan_gazebo')
    pkg_slam = FindPackageShare('aetherscan_slam')
    pkg_nav = FindPackageShare('aetherscan_navigation')
    pkg_control = FindPackageShare('aetherscan_control')
    pkg_mission = FindPackageShare('aetherscan_mission')
    pkg_bringup = FindPackageShare('aetherscan_bringup')

    world_arg = DeclareLaunchArgument(
        'world', default_value='office_environment',
        description='Gazebo world to load'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )
    rosbridge_arg = DeclareLaunchArgument(
        'rosbridge', default_value='true',
        description='Launch rosbridge for web dashboard'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_slam, 'launch', 'slam.launch.py'])
        ])
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])
        ])
    )

    control_params = PathJoinSubstitution([pkg_control, 'config', 'control_params.yaml'])

    flight_controller = Node(
        package='aetherscan_control',
        executable='flight_controller',
        name='flight_controller',
        parameters=[control_params, {'use_sim_time': True}],
        output='screen'
    )

    trajectory_tracker = Node(
        package='aetherscan_control',
        executable='trajectory_tracker',
        name='trajectory_tracker',
        parameters=[control_params, {'use_sim_time': True}],
        output='screen'
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mission, 'launch', 'mission.launch.py'])
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_bringup, 'config', 'rviz', 'aetherscan.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'unregister_timeout': 30.0,
        }],
        condition=IfCondition(LaunchConfiguration('rosbridge'))
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        rosbridge_arg,
        gazebo_launch,
        slam_launch,
        nav_launch,
        flight_controller,
        trajectory_tracker,
        mission_launch,
        rviz_node,
        rosbridge_node,
    ])
