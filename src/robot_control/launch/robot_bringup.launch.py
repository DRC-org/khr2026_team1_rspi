import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_control = get_package_share_directory('robot_control')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Configuration Files
    slam_config_path = os.path.join(pkg_robot_control, 'config', 'mapper_params_online_async.yaml')
    nav2_params_path = os.path.join(pkg_robot_control, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_robot_control, 'rviz', 'nav2.rviz')
    map_file_path = os.path.join(pkg_robot_control, 'maps', 'field.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_mode = LaunchConfiguration('map_mode')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_mode = DeclareLaunchArgument(
        'map_mode',
        default_value='true',
        description='True: Use static map (AMCL), False: Use SLAM (Mapping)')

    # ---------------------------------------------------------
    # Common Nodes
    # ---------------------------------------------------------

    # YDLidar Launch
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py')
        )
    )

    # Static Transform Publisher (base_link -> laser_frame)
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Temporary static TF (map -> odom) until AMCL initializes
    # AMCL will override this once initial pose is set
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Bluetooth Node
    bt_node = Node(
        package='bt_communication',
        executable='bluetooth_node',
        name='bluetooth_node',
        output='screen',
        parameters=[{'hci_transport': 'usb:0'}]
    )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # ---------------------------------------------------------
    # Nav2 Nodes (Shared between modes)
    # ---------------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_path],
        remappings=[('/cmd_vel', '/cmd_vel_nav')])

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path])

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path])

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path])

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_path])

    # ---------------------------------------------------------
    # Conditional Nodes (based on map_mode)
    # ---------------------------------------------------------

    # [SLAM Mode] SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'params_file': slam_config_path, 'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(map_mode)
    )

    # [Map Mode] Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path},
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(map_mode)
    )

    # [Map Mode] AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(map_mode)
    )

    # ---------------------------------------------------------
    # Lifecycle Managers
    # ---------------------------------------------------------
    
    # Node names to manage
    lifecycle_nodes_slam = ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator',
                           'waypoint_follower']
                           
    lifecycle_nodes_map = ['map_server',
                           'amcl',
                           'controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator',
                           'waypoint_follower']

    # Manager for SLAM Mode
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes_slam}],
        condition=UnlessCondition(map_mode)
    )

    # Manager for Map Mode
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes_map}],
        condition=IfCondition(map_mode)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_mode,
        
        tf_base_laser,
        tf_map_odom,
        ydlidar_launch,
        bt_node,
        rviz_node,
        
        # Shared Nav2 Nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        
        # SLAM specific
        slam_launch,
        lifecycle_manager_slam,
        
        # Map specific
        map_server_node,
        amcl_node,
        lifecycle_manager_map,
    ])
