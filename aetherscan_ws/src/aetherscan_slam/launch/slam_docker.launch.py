"""Lightweight SLAM for Docker (no RTAB-Map GPU dependency)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    point_cloud_assembler = Node(
        package='aetherscan_slam',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        parameters=[{
            'use_sim_time': True,
            'voxel_size': 0.03,
            'max_points': 2000000,
            'max_range': 8.0,
            'min_range': 0.3,
            'map_frame': 'map',
            'publish_rate': 2.0,
        }],
        output='screen',
    )

    map_manager = Node(
        package='aetherscan_slam',
        executable='map_manager',
        name='map_manager',
        parameters=[{
            'use_sim_time': True,
            'save_directory': '/tmp/aetherscan_maps',
            'auto_save_interval': 120.0,
        }],
        output='screen',
    )

    return LaunchDescription([point_cloud_assembler, map_manager])
