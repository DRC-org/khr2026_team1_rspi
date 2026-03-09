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
    pkg_share_auto_nav = get_package_share_directory('auto_nav')

    xacro_file = os.path.join(pkg_share_sim, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share_sim, 'worlds', 'khr2026_field.sdf')
    rviz_config = os.path.join(pkg_share_auto_nav, 'config', 'khr2026_team1.rviz')
    map_yaml = os.path.join(pkg_share_sim, 'maps', 'field_synthetic.yaml')

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

    # 3. Spawn robot in Gazebo（Y 反転済みワールド: 南が y=7 なので南側は -y 6.5）
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot',
                   '-x', '3.0', '-y', '6.5', '-z', '0.05', '-Y', '-1.5708'],
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

    # 5. map と odom を一致させる静的 TF（シミュでは同一ワールドとする）
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    # 6. map_server（/map 配信、RViz で壁表示）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_share_sim, 'config', 'lifecycle_map_server.yaml'),
        ],
    )

    # 7. cmd_vel inverter (cmd_vel_nav -> cmd_vel_inverted)
    cmd_vel_inverter = Node(
        package='khr2026_team1_simulation',
        executable='invert_cmd_vel.py',
        name='cmd_vel_inverter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. RViz2
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
        static_tf_map_odom,
        map_server_node,
        lifecycle_manager_map,
        cmd_vel_inverter,
        rviz,
    ])
