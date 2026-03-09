"""
自律走行用ランチファイル（AMCL ローカリゼーションモード）

事前にフィールドマップを生成してから使用する:
  ros2 run auto_nav generate_field_map.py
  → /home/pi/maps/field_synthetic.pgm / field_synthetic.yaml が生成される

使い方:
  ros2 launch auto_nav auto_nav_launch.py
  ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field_synthetic.yaml
  ros2 launch auto_nav auto_nav_launch.py transport:=udp4

起動するノード:
  - micro_ros_agent (cwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_c)
  - micro_ros_agent (hwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_h)
  - ydlidar_ros2_driver    (LiDAR)
  - static_tf              (base_link → laser_frame)
  - laser_filter           (/scan → /scan_filtered)
  - odometry_node          (wheel_feedback → /odom_raw, エンコーダのみ)
  - imu_publisher_node     (wheel_feedback → /imu, gyro/accel)
  - ekf_filter_node        (/odom_raw + /imu → /odom + TF(odom→base_link))
  - cmd_vel_bridge_node    (/cmd_vel → wheel_control, auto モード時のみ)
  - map_server             (合成 PGM マップ → /map, lifecycle)
  - amcl                   (既知マップでのパーティクルフィルタ自己位置推定, lifecycle)
  - lifecycle_manager_localization (map_server + amcl のライフサイクル管理)
  - nav2_bringup           (controller_server / planner_server / behavior_server / bt_navigator)
  - routing_node           (Bluetooth → NavigateToPose, nav_mode 切り替え)
  - robot_control          (Bluetooth 手動操縦 + ESP32 制御)
  - bt_communication       (Bluetooth GATT サーバー)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction

_BT_VENV_SITE_PACKAGES = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "../../../../../src/bt_communication/.venv/lib/python3.12/site-packages",
    )
)
_existing = os.environ.get("PYTHONPATH", "")
_BT_PYTHONPATH = _BT_VENV_SITE_PACKAGES + (":" + _existing if _existing else "")

_WEB_VENV_SITE_PACKAGES = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "../../../../../src/web_control/.venv/lib/python3.12/site-packages",
    )
)
_WEB_PYTHONPATH = _WEB_VENV_SITE_PACKAGES + (":" + _existing if _existing else "")


def generate_launch_description():
    auto_nav_share = get_package_share_directory("auto_nav")
    laser_filters_config = os.path.join(auto_nav_share, "config", "laser_filters.yaml")
    nav2_params = os.path.join(auto_nav_share, "config", "nav2_params.yaml")
    ekf_params = os.path.join(auto_nav_share, "config", "ekf_params.yaml")
    amcl_params = os.path.join(auto_nav_share, "config", "amcl_params.yaml")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/pi/maps/field_synthetic.yaml",
        description="map_server に渡すマップ YAML パス。例: /home/pi/maps/field_synthetic.yaml",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="詳細ログを有効にする ('true' で --log-level debug を各ノードに適用)",
    )
    _log_level = PythonExpression(
        ['"debug" if "', LaunchConfiguration("debug"), '" == "true" else "info"']
    )

    transport_arg = DeclareLaunchArgument(
        "transport",
        default_value="serial",
        description="micro-ROS agent の接続方式: 'serial'（USB, /dev/esp32_c + /dev/esp32_h）または 'udp4'（Wi-Fi, cwmc:8888 / hwmc:8889）",
    )

    _is_serial = IfCondition(PythonExpression(['"', LaunchConfiguration("transport"), '" == "serial"']))
    _is_udp = IfCondition(PythonExpression(['"', LaunchConfiguration("transport"), '" == "udp4"']))

    # シリアル接続（transport:=serial, デフォルト）
    micro_ros_agent_serial_c = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_c",
        arguments=["serial", "--dev", "/dev/esp32_c"],
        output="screen",
        emulate_tty=True,
        condition=_is_serial,
    )

    micro_ros_agent_serial_h = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_h",
        arguments=["serial", "--dev", "/dev/esp32_h"],
        output="screen",
        emulate_tty=True,
        condition=_is_serial,
    )

    # UDP 接続（transport:=udp4, Wi-Fi 経由）
    micro_ros_agent_udp_c = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_c",
        arguments=["udp4", "--port", "8888"],
        output="screen",
        emulate_tty=True,
        condition=_is_udp,
    )

    micro_ros_agent_udp_h = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_h",
        arguments=["udp4", "--port", "8889"],
        output="screen",
        emulate_tty=True,
        condition=_is_udp,
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ydlidar_ros2_driver"),
                "launch",
                "ydlidar_launch.py",
            )
        )
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.15",
            "--yaw", "1.5708", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser_frame",
        ],
        output="screen",
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filters_config],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
        output="screen",
        emulate_tty=True,
    )

    odometry_node = Node(
        package="auto_nav",
        executable="launch_odometry.py",
        name="odometry_node",
        output="screen",
        emulate_tty=True,
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    imu_publisher_node = Node(
        package="auto_nav",
        executable="launch_imu_publisher.py",
        name="imu_publisher_node",
        output="screen",
        emulate_tty=True,
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        emulate_tty=True,
        parameters=[ekf_params],
        remappings=[("odometry/filtered", "/odom")],
    )

    cmd_vel_bridge_node = Node(
        package="auto_nav",
        executable="launch_cmd_vel_bridge.py",
        name="cmd_vel_bridge_node",
        output="screen",
        emulate_tty=True,
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    # 合成 PGM マップを /map トピックで配信（lifecycle ノード）
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"yaml_filename": LaunchConfiguration("map")},
            {"use_sim_time": False},
        ],
    )

    # 既知マップでのパーティクルフィルタ自己位置推定（lifecycle ノード）
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[amcl_params],
    )

    # map_server + amcl のライフサイクル管理
    lifecycle_manager_localization_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[nav2_params],
    )

    # Nav2 必要ノードのみを個別起動（navigation_launch.py は不要ノードを多数含むため使わない）
    # smoother_server / route_server / waypoint_follower / docking_server は競技不使用のため除外
    # velocity_smoother / collision_monitor も除外し、controller_server が /cmd_vel へ直接出力
    _nav2_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=_nav2_remaps,
        # cmd_vel リマップなし → /cmd_vel へ直接出力
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        output="screen",
        parameters=[nav2_params],
        remappings=_nav2_remaps,
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output="screen",
        parameters=[nav2_params],
        remappings=_nav2_remaps,
        # cmd_vel リマップなし → /cmd_vel へ直接出力
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        output="screen",
        parameters=[nav2_params],
        remappings=_nav2_remaps,
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav2_params],
    )

    routing_node = Node(
        package="auto_nav",
        executable="launch_routing.py",
        name="routing_node",
        output="screen",
        emulate_tty=True,
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    robot_control_node = Node(
        package="robot_control",
        executable="launch.py",
        name="robot_control_node",
        output="screen",
        emulate_tty=True,
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    bt_communication_node = Node(
        package="bt_communication",
        executable="launch.py",
        name="bt_communication_node",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONPATH": _BT_PYTHONPATH},
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    web_server_node = Node(
        package="web_control",
        executable="launch_web_server.py",
        name="web_server_node",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONPATH": _WEB_PYTHONPATH},
        ros_arguments=["--log-level", _log_level, "--log-level", "rcl:=warn", "--log-level", "rclpy:=warn"],
    )

    return LaunchDescription(
        [
            map_arg,
            debug_arg,
            transport_arg,
            micro_ros_agent_serial_c,
            micro_ros_agent_serial_h,
            micro_ros_agent_udp_c,
            micro_ros_agent_udp_h,
            ydlidar_launch,
            static_tf_node,
            laser_filter_node,
            odometry_node,
            imu_publisher_node,
            ekf_node,
            map_server_node,
            amcl_node,
            # EKF が odom→base_link TF を発行し始めるまで AMCL 起動を遅延
            # （TF なしで起動するとスキャンキューが満杯になり AMCL が LiDAR を一切処理できなくなる）
            TimerAction(period=8.0, actions=[lifecycle_manager_localization_node]),
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            # EKF の TF(odom→base_link) + localization 起動後に activate するため遅延
            TimerAction(period=15.0, actions=[lifecycle_manager_node]),
            routing_node,
            robot_control_node,
            bt_communication_node,
            web_server_node,
            cmd_vel_bridge_node,
        ]
    )
