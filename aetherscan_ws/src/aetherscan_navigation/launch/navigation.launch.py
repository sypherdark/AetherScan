from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = FindPackageShare('aetherscan_navigation')
    params_file = PathJoinSubstitution([pkg_nav, 'config', 'navigation_params.yaml'])

    frontier_explorer = Node(
        package='aetherscan_navigation',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    path_planner = Node(
        package='aetherscan_navigation',
        executable='path_planner',
        name='path_planner',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    obstacle_avoidance = Node(
        package='aetherscan_navigation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        frontier_explorer,
        path_planner,
        obstacle_avoidance,
    ])
