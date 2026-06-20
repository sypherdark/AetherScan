from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg = FindPackageShare('aetherscan_perception')
    params = PathJoinSubstitution([pkg, 'config', 'perception_params.yaml'])

    processor = Node(
        package='aetherscan_perception',
        executable='point_cloud_processor',
        name='point_cloud_processor',
        parameters=[params, {'use_sim_time': True}],
        output='screen'
    )

    mesh = Node(
        package='aetherscan_perception',
        executable='mesh_reconstructor',
        name='mesh_reconstructor',
        parameters=[params, {'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([processor, mesh])
