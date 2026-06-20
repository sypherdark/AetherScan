"""Launch autonomous scanning mission.

Starts the full simulation and automatically begins the scanning mission.
"""

import time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('aetherscan_bringup')

    world_arg = DeclareLaunchArgument(
        'world', default_value='office_environment',
        description='Gazebo world to load'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_bringup, 'launch', 'simulation.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': 'true',
            'rosbridge': 'true',
        }.items()
    )

    start_mission = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/aetherscan/start_mission',
                     'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        simulation,
        start_mission,
    ])
