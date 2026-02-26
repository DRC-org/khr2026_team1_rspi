"""
マッピング用ランチファイル

起動するノード:
  - micro_ros_agent        (ESP32 ↔ ROS2 ブリッジ, /dev/ttyUSB0)
  - ydlidar_ros2_driver    (LiDAR)
  - static_tf              (base_link → laser_frame)
  - laser_filter           (/scan → /scan_filtered, タイヤ映り込み除去)
  - slam_toolbox           (mapping モード, /scan_filtered + /odom → /map + TF)
  - robot_control          (Bluetooth手動操縦 + ESP32制御)
  - bt_communication       (Bluetooth GATT サーバー)
  - auto_nav odometry      (wheel_feedback → /odom + TF)
  - auto_nav cmd_vel_bridge(/cmd_vel → wheel_control, auto モード時のみ有効)

使い方:
  ros2 launch auto_nav mapping_launch.py
  ros2 launch auto_nav mapping_launch.py serial_port:=/dev/ttyUSB1  # ポートを変える場合
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

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="シリアル接続時の ESP32 のポート（transport:=serial のときのみ使用）",
    )

    transport_arg = DeclareLaunchArgument(
        "transport",
        default_value="serial",
        description="micro-ROS agent の接続方式: 'serial'（USB）または 'udp4'（Wi-Fi, port 8888）",
    )

    # シリアル接続（transport:=serial, デフォルト）
    micro_ros_agent_serial = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", LaunchConfiguration("serial_port")],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration("transport"), '" == "serial"'])),
    )

    # UDP 接続（transport:=udp4, Wi-Fi 経由）
    micro_ros_agent_udp = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["udp4", "--port", "8888"],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration("transport"), '" == "udp4"'])),
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
    )

    bt_communication_node = Node(
        package="bt_communication",
        executable="launch.py",
        name="bt_communication_node",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONPATH": _BT_PYTHONPATH},
    )

    odometry_node = Node(
        package="auto_nav",
        executable="launch_odometry.py",
        name="odometry_node",
        output="screen",
        emulate_tty=True,
    )

    cmd_vel_bridge_node = Node(
        package="auto_nav",
        executable="launch_cmd_vel_bridge.py",
        name="cmd_vel_bridge_node",
        output="screen",
        emulate_tty=True,
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
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            transport_arg,
            micro_ros_agent_serial,
            micro_ros_agent_udp,
            ydlidar_launch,
            static_tf_node,
            laser_filter_node,
            odometry_node,
            slam_toolbox_node,
            slam_lifecycle,
            robot_control_node,
            bt_communication_node,
            cmd_vel_bridge_node,
        ]
    )
