import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share_sim = get_package_share_directory('khr2026_team1_simulation')
    pkg_share_control = get_package_share_directory('robot_control')

    xacro_file = os.path.join(pkg_share_sim, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share_sim, 'worlds', 'khr2026_field.sdf')
    rviz_config = os.path.join(pkg_share_control, 'rviz', 'sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # 2. Gazebo Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': [f'-v 4 -r {world_file}']}.items(),
    )

    # 3. Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot',
                   '-x', '3.0', '-y', '0.5', '-z', '0.05', '-Y', '-1.5708'],
        output='screen'
    )

    # 4. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share_sim, 'config', 'bridge.yaml'),
        }],
        output='screen'
    )

    # 5. cmd_vel inverter (cmd_vel_nav -> cmd_vel_inverted)
    cmd_vel_inverter = Node(
        package='khr2026_team1_simulation',
        executable='invert_cmd_vel.py',
        name='cmd_vel_inverter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        cmd_vel_inverter,
        rviz,
    ])
