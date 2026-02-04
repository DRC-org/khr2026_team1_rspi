import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_robot_control = get_package_share_directory('robot_control')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Configuration Files
    slam_config_path = os.path.join(pkg_robot_control, 'config', 'mapper_params_online_async.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # YDLidar Launch
    # Assuming ydlidar_ros2_driver has a launch file named ydlidar_launch.py
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py')
        )
    )

    # Static Transform Publisher (base_link -> laser_frame)
    # Adjust args as per actual robot mounting (x y z yaw pitch roll)
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Fake Odometry (odom -> base_link)
    # WARNING: This is only for testing without a real robot base!
    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'params_file': slam_config_path, 'use_sim_time': use_sim_time}.items()
    )

    # Nav2 Bringup (Navigation)
    nav2_params_path = os.path.join(pkg_robot_control, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_robot_control, 'rviz', 'nav2.rviz')
    
    # Only launch core navigation nodes to avoid configuration errors
    # We launch them manually instead of using navigation_launch.py to have full control
    
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']
                       
    nav2_group = GroupAction([
        PushRosNamespace('/'),
        
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_path],
            arguments=['--ros-args', '--log-level', 'info']),
            
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path],
            arguments=['--ros-args', '--log-level', 'info']),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path],
            arguments=['--ros-args', '--log-level', 'info']),
            
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path],
            arguments=['--ros-args', '--log-level', 'info']),
            
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_path],
            arguments=['--ros-args', '--log-level', 'info']),
            
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        tf_base_laser,
        tf_odom_base,
        ydlidar_launch,
        slam_launch,
        nav2_group,
        rviz_node
    ])
