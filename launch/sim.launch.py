import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_vacuum_bot = get_package_share_directory('vacuum_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    urdf_file = os.path.join(pkg_vacuum_bot, 'urdf', 'vacuum.urdf')
    world_file = os.path.join(pkg_vacuum_bot, 'worlds', 'maze.sdf')
    bridge_config = os.path.join(pkg_vacuum_bot, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_vacuum_bot, 'config', 'rviz.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'vacuum', '-string', robot_desc, '-z', '0.2'],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    mapper_node = Node(
        package='vacuum_bot',
        executable='simple_mapper.py',
        name='simple_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        gz_bridge,
        rviz2,
        mapper_node
    ])
