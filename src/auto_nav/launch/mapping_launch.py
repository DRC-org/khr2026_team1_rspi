"""
マッピング用ランチファイル

起動するノード:
  - micro_ros_agent        (ESP32 ↔ ROS2 ブリッジ, /dev/ttyUSB0)
  - ydlidar_ros2_driver    (LiDAR)
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="micro-ROS agent に接続する ESP32 のシリアルポート",
    )

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", LaunchConfiguration("serial_port")],
        output="screen",
        emulate_tty=True,
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

    return LaunchDescription(
        [
            serial_port_arg,
            micro_ros_agent_node,
            ydlidar_launch,
            robot_control_node,
            bt_communication_node,
            odometry_node,
            cmd_vel_bridge_node,
        ]
    )
