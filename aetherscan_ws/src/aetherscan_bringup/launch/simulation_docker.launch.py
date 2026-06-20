"""Docker / macOS launch: indoor physics simulator + full stack (no Gazebo GUI)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg_gazebo = FindPackageShare('aetherscan_gazebo')
    pkg_slam = FindPackageShare('aetherscan_slam')
    pkg_nav = FindPackageShare('aetherscan_navigation')
    pkg_control = FindPackageShare('aetherscan_control')
    pkg_mission = FindPackageShare('aetherscan_mission')
    pkg_bringup = FindPackageShare('aetherscan_bringup')

    world_arg = DeclareLaunchArgument(
        'world', default_value='office_environment',
        description='office_environment | warehouse'
    )

    indoor_sim = Node(
        package='aetherscan_gazebo',
        executable='indoor_simulator',
        name='indoor_simulator',
        parameters=[{
            'use_sim_time': True,
            'world': LaunchConfiguration('world'),
            'publish_rate': 50.0,
            'lidar_rate': 10.0,
        }],
        output='screen',
    )

    xacro_path = PathJoinSubstitution([
        FindPackageShare('aetherscan_description'),
        'urdf',
        'aetherscan_drone.urdf.xacro',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]), value_type=str
            ),
        }],
        output='screen',
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_slam, 'launch', 'slam_docker.launch.py'])
        ])
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])
        ])
    )

    control_params = PathJoinSubstitution([
        FindPackageShare('aetherscan_control'), 'config', 'control_params.yaml'
    ])

    flight_controller = Node(
        package='aetherscan_control',
        executable='flight_controller',
        name='flight_controller',
        parameters=[control_params, {'use_sim_time': True}],
        output='screen',
    )

    trajectory_tracker = Node(
        package='aetherscan_control',
        executable='trajectory_tracker',
        name='trajectory_tracker',
        parameters=[control_params, {'use_sim_time': True}],
        output='screen',
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mission, 'launch', 'mission.launch.py'])
        ])
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        indoor_sim,
        robot_state_publisher,
        slam_launch,
        nav_launch,
        flight_controller,
        trajectory_tracker,
        mission_launch,
        rosbridge,
    ])
