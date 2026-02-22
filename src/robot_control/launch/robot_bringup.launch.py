import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    pkg_robot_control = get_package_share_directory("robot_control")
    # pkg_ydlidar = get_package_share_directory("ydlidar_ros2_driver")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")

    # Configuration Files
    slam_config_path = os.path.join(
        pkg_robot_control, "config", "mapper_params_online_async.yaml"
    )
    nav2_params_path = os.path.join(pkg_robot_control, "config", "nav2_params.yaml")
    rviz_config_path = os.path.join(pkg_robot_control, "rviz", "nav2.rviz")
    map_file_path = os.path.join(pkg_robot_control, "maps", "field.yaml")
    ekf_config_path = os.path.join(pkg_robot_control, "config", "ekf.yaml")

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_mode = LaunchConfiguration("map_mode")
    use_sim_data = LaunchConfiguration("use_sim_data")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_map_mode = DeclareLaunchArgument(
        "map_mode",
        default_value="true",
        description="True: Use static map (AMCL), False: Use SLAM (Mapping)",
    )

    declare_use_sim_data = DeclareLaunchArgument(
        "use_sim_data",
        default_value="false",
        description="true: simulation mode (Gazebo), false: real robot mode",
    )

    # ---------------------------------------------------------
    # Common Nodes
    # ---------------------------------------------------------

    # YDLidar Launch
    # ydlidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ydlidar, "launch", "ydlidar_launch.py")
    #     )
    # )

    # Static Transform Publisher (base_link -> laser_frame)
    # Note: Handled by robot_state_publisher via URDF
    # tf_base_laser = Node(...)

    # Static Transform Publisher (odom -> base_link)
    # tf_odom_base = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    #     output="screen",
    # )

    # Temporary static TF (map -> odom) until AMCL initializes
    # AMCL will override this once initial pose is set
    tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom", "--ros-args", "--log-level", "ERROR"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(map_mode),
    )

    # Bluetooth Node は実機モードのみ
    bt_node = Node(
        package="bt_communication",
        executable="bluetooth_node",
        name="bluetooth_node",
        output="screen",
        parameters=[{"hci_transport": "usb:0"}],
        condition=UnlessCondition(use_sim_data),
    )

    # SimMonitor はシミュレーションモードのみ (到達通知・表示)
    sim_monitor_node = Node(
        package="robot_control",
        executable="sim_monitor.py",
        name="sim_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(use_sim_data),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )

    # Rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path, "--ros-args", "--log-level", "ERROR"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---------------------------------------------------------
    # Nav2 Nodes (Shared between modes)
    # ---------------------------------------------------------
    # Real robot: remap /cmd_vel → /cmd_vel_nav to avoid conflicts with hardware driver
    # Simulation: output directly to /cmd_vel which Gazebo bridge listens to
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )


    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
    )

    # ---------------------------------------------------------
    # Conditional Nodes (based on map_mode)
    # ---------------------------------------------------------

    # [SLAM Mode] SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "params_file": slam_config_path,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=UnlessCondition(map_mode),
    )

    # [Map Mode] Map Server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params_path,
            {"yaml_filename": map_file_path},
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(map_mode),
    )

    # [Map Mode] AMCL
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_path, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(map_mode),
    )

    # ---------------------------------------------------------
    # Lifecycle Managers
    # ---------------------------------------------------------

    # Node names to manage
    lifecycle_nodes_slam = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    lifecycle_nodes_map = [
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    # Manager for SLAM Mode
    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes_slam},
        ],
        condition=UnlessCondition(map_mode),
    )

    # Manager for Map Mode
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes_map},
        ],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(map_mode),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map_mode,
            declare_use_sim_data,
            # tf_base_laser, # Removed redundant TF
            # tf_odom_base,
            tf_map_odom,
            # ydlidar_launch,
            bt_node,
            sim_monitor_node,
            robot_localization_node,
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
        ]

    )
