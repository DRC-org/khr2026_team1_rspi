"""
自律走行用ランチファイル（ローカリゼーションモード）

事前にマッピングを完了し、地図を保存してから使用する:
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "filename: '/home/taiga/maps/field'"
  → field.posegraph / field.data が生成される

使い方:
  ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field
  ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field transport:=udp4

起動するノード:
  - micro_ros_agent (cwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_c)
  - micro_ros_agent (hwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_h)
  - ydlidar_ros2_driver    (LiDAR)
  - static_tf              (base_link → laser_frame)
  - laser_filter           (/scan → /scan_filtered)
  - odometry_node          (wheel_feedback → /odom_raw, エンコーダのみ)
  - imu_publisher_node     (wheel_feedback → /imu, gyro/accel)
  # scan_odometry_node は CPU 負荷軽減のため除去（EKF odom1 が全 false のため不要）
  - ekf_filter_node        (/odom_raw + /imu → /odom + TF(odom→base_link))
  - cmd_vel_bridge_node    (/cmd_vel → wheel_control, auto モード時のみ)
  - slam_toolbox           (localization モード, 保存済み地図で自己位置推定)
  - nav2_bringup           (controller_server / planner_server / behavior_server / bt_navigator)
  - routing_node           (Bluetooth → NavigateToPose, nav_mode 切り替え)
  - robot_control          (Bluetooth 手動操縦 + ESP32 制御)
  - bt_communication       (Bluetooth GATT サーバー)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

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
    slam_params = os.path.join(auto_nav_share, "config", "slam_localization_params.yaml")
    nav2_params = os.path.join(auto_nav_share, "config", "nav2_params.yaml")
    ekf_params = os.path.join(auto_nav_share, "config", "ekf_params.yaml")

    map_arg = DeclareLaunchArgument(
        "map",
        description="保存済みマップのパス（拡張子なし）。例: /home/taiga/maps/field",
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

    scan_relay_node = Node(
        package="auto_nav",
        executable="launch_scan_relay.py",
        name="scan_relay_node",
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

    # slam_toolbox をローカリゼーションモードで起動
    # map_file_name はローンチ引数 map:= で上書きする
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_params,
            {"map_file_name": LaunchConfiguration("map")},
        ],
        output="screen",
        emulate_tty=True,
    )

    # マッピング時と同様に ros2 service call でライフサイクル管理
    slam_lifecycle = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "ros2 service call /slam_toolbox/change_state"
                    " lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}'"
                    " && sleep 2"
                    " && ros2 service call /slam_toolbox/change_state"
                    " lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'",
                ],
                output="screen",
            )
        ],
    )

    # nav2_bringup/navigation_launch.py: controller_server / planner_server /
    # behavior_server / bt_navigator + lifecycle_manager_navigation を起動する。
    # bringup_launch.py（map_server + AMCL 含む）は使わない。
    # /map は slam_toolbox が配信済みのため不要。
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
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
            scan_relay_node,
            odometry_node,
            imu_publisher_node,
            ekf_node,
            slam_toolbox_node,
            slam_lifecycle,
            nav2_launch,
            routing_node,
            robot_control_node,
            bt_communication_node,
            web_server_node,
            cmd_vel_bridge_node,
        ]
    )
