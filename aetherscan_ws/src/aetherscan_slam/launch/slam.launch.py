import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = FindPackageShare('aetherscan_slam')

    rtabmap_params = PathJoinSubstitution([
        pkg_slam, 'config', 'rtabmap_params.yaml'
    ])

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', '/aetherscan/camera/image'),
            ('depth/image', '/aetherscan/camera/depth'),
            ('rgb/camera_info', '/aetherscan/camera/camera_info'),
            ('scan_cloud', '/aetherscan/lidar'),
            ('odom', '/aetherscan/odom'),
        ],
        arguments=['--delete_db_on_start'],
        output='screen'
    )

    point_cloud_assembler = Node(
        package='aetherscan_slam',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        parameters=[{
            'use_sim_time': True,
            'voxel_size': 0.03,
            'max_points': 5000000,
            'max_range': 8.0,
            'min_range': 0.3,
            'map_frame': 'map',
            'publish_rate': 2.0,
        }],
        output='screen'
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
        output='screen'
    )

    return LaunchDescription([
        rtabmap_node,
        point_cloud_assembler,
        map_manager,
    ])
