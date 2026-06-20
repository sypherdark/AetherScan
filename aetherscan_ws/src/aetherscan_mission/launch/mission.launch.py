from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mission = FindPackageShare('aetherscan_mission')
    params_file = PathJoinSubstitution([pkg_mission, 'config', 'mission_params.yaml'])

    mission_controller = Node(
        package='aetherscan_mission',
        executable='mission_controller',
        name='mission_controller',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([mission_controller])
