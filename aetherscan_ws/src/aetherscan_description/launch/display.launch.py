from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('aetherscan_description')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                pkg, 'urdf', 'aetherscan_drone.urdf.xacro'
            ])
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('aetherscan_bringup'),
            'config', 'rviz', 'aetherscan.rviz'
        ])]
    )

    return LaunchDescription([robot_state_publisher, rviz])
