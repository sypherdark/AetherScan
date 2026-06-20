from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg = FindPackageShare('aetherscan_control')
    params = PathJoinSubstitution([pkg, 'config', 'control_params.yaml'])

    flight_controller = Node(
        package='aetherscan_control',
        executable='flight_controller',
        name='flight_controller',
        parameters=[params, {'use_sim_time': True}],
        output='screen'
    )

    trajectory_tracker = Node(
        package='aetherscan_control',
        executable='trajectory_tracker',
        name='trajectory_tracker',
        parameters=[params, {'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([flight_controller, trajectory_tracker])
