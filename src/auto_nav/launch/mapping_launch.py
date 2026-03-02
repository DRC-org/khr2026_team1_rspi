"""
マッピング用ランチファイル

起動するノード:
  - micro_ros_agent (cwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_c)
  - micro_ros_agent (hwmc)  (ESP32 ↔ ROS2 ブリッジ, /dev/esp32_h)
  - ydlidar_ros2_driver    (LiDAR)
  - static_tf              (base_link → laser_frame)
  - laser_filter           (/scan → /scan_filtered, タイヤ映り込み除去)
  - slam_toolbox           (mapping モード, /scan_filtered + /odom → /map + TF)
  - robot_control          (Bluetooth手動操縦 + ESP32制御)
  - bt_communication       (Bluetooth GATT サーバー)
  - auto_nav odometry      (wheel_feedback → /odom_raw, エンコーダのみ)
  - auto_nav imu_publisher (wheel_feedback → /imu, gyro/accel)
  - scan_odometry_node     (/scan_filtered → /scan_odom, LiDAR scan-to-scan)
  - ekf_filter_node        (/odom_raw + /imu + /scan_odom → /odom + TF(odom→base_link))
  - auto_nav cmd_vel_bridge(/cmd_vel → wheel_control, auto モード時のみ有効)

使い方:
  ros2 launch auto_nav mapping_launch.py
  ros2 launch auto_nav mapping_launch.py transport:=udp4  # Wi-Fi 接続（cwmc:8888, hwmc:8889）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# bt_communication は bumble を .venv で管理しているため、
# そのノードにだけ PYTHONPATH を通してシステム Python から bumble を参照できるようにする
# __file__ は install/auto_nav/share/auto_nav/launch/ に置かれるため、
# 5段上がると workspace root になる
_BT_VENV_SITE_PACKAGES = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "../../../../../src/bt_communication/.venv/lib/python3.12/site-packages",
    )
)

# 既存の PYTHONPATH を保ちつつ venv を先頭に追加する
# (上書きすると bt_communication モジュール自体が見えなくなる)
_existing = os.environ.get("PYTHONPATH", "")
_BT_PYTHONPATH = _BT_VENV_SITE_PACKAGES + (":" + _existing if _existing else "")


def generate_launch_description():
    auto_nav_share = get_package_share_directory("auto_nav")
    laser_filters_config = os.path.join(auto_nav_share, "config", "laser_filters.yaml")
    slam_params = os.path.join(auto_nav_share, "config", "slam_mapping_params.yaml")
    ekf_params = os.path.join(auto_nav_share, "config", "ekf_params.yaml")

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

    # LiDAR scan-to-scan オドメトリ（icp_odometry, rtabmap_odom）
    # /scan_filtered からスキャンマッチングで絶対姿勢を推定し /scan_odom に配信する。
    # EKF が yaw を差分モードで取り込み、エンコーダ/IMU と融合する。
    scan_odometry_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="scan_odometry_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "subscribe_scan": True,
            "publish_tf": False,           # TF は EKF が担当
            "Odom/Strategy": "0",          # Frame-to-Frame（scan-to-scan）
            "Odom/GuessMotion": "true",    # 前フレームの速度を初期推定に利用
            "Icp/PointToPlane": "false",   # 2D スキャン（ノーマルなし）
            "Icp/MaxCorrespondenceDistance": "0.5",
            "Icp/Iterations": "10",
            "Icp/MaxTranslation": "1.0",
            "Icp/MaxRotation": "0.785",    # 45°/update を上限
        }],
        remappings=[
            ("scan", "/scan_filtered"),
            ("odom", "/scan_odom"),
        ],
        ros_arguments=["--log-level", "warn"],
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

    # LiDAR の TF: base_link → laser_frame
    # 値はロボット実機のLiDAR取付位置に合わせて調整すること
    #   x, y: ロボット中心からの水平オフセット [m]（中心なら 0.0, 0.0）
    #   z: 取付高さ [m]（地面からではなくロボット基準座標からの高さ）
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.15",
            "--yaw", "1.5708", "--pitch", "3.14159", "--roll", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser_frame",
        ],
        output="screen",
    )

    # タイヤ映り込みフィルタ: /scan → /scan_filtered
    # ロボット中心から 0.32m 以内の点を除去（タイヤは ~0.30m）
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

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
        emulate_tty=True,
    )

    # slam_toolbox はライフサイクルノードだが ros2 lifecycle コマンドはデーモン経由で
    # 検出できないため、ros2 service call でサービスを直接呼び出す。
    # ros2 service call はサービスが現れるまで自動で待機するため、タイマーの初期遅延のみ必要。
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
                output="log",  # Ctrl+C 時の KeyboardInterrupt トレースバックを端末に出さない
            )
        ],
    )

    return LaunchDescription(
        [
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
            scan_odometry_node,
            ekf_node,
            slam_toolbox_node,
            slam_lifecycle,
            robot_control_node,
            bt_communication_node,
            cmd_vel_bridge_node,
        ]
    )
