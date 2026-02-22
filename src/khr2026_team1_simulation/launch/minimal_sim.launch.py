import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. パッケージパスの取得
    pkg_share_sim = get_package_share_directory('khr2026_team1_simulation')
    pkg_share_control = get_package_share_directory('robot_control')
    
    xacro_file = os.path.join(pkg_share_sim, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share_sim, 'worlds', 'khr2026_field.sdf')
    rviz_config_path = os.path.join(pkg_share_control, 'rviz', 'nav2.rviz')
    ekf_config_path = os.path.join(pkg_share_control, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Robot State Publisher (URDFの配信)
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

    # 3. Gazebo Simulator (Physics Server)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': [f'-v 4 -r -s {world_file}']}.items(),
    )

    # 4. Robot Spawn (Gazebo内に召還)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot', '-x', '0.8', '-y', '1.0', '-z', '0.1'],
        output='screen'
    )

    # 5. ROS-Gazebo Bridge (Topicの橋渡し)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share_sim, 'config', 'bridge.yaml'),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 6. EKF Node (odom -> base_footprint の TF 生成)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # 7. Cmd_Vel Inverter (実機の反転仕様に合わせた補正)
    cmd_vel_inverter = Node(
        package='khr2026_team1_simulation',
        executable='invert_cmd_vel.py',
        name='cmd_vel_inverter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. RViz2 (可視化)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        ekf_node,
        cmd_vel_inverter,
        rviz_node
    ])
