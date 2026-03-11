import sys
from pathlib import Path

# robot_control パッケージの __init__.py は rclpy/robot_msgs に依存するため、
# テスト時は個別サブモジュール（vec2, m3508, heading_pid, constants）を
# パッケージ経由ではなく直接 import できるようにする。
_SRC_DIR = Path(__file__).resolve().parent.parent / "src" / "robot_control"

# sys.modules に空パッケージとして登録し、__init__.py の実行を回避
import types
_pkg = types.ModuleType("robot_control")
_pkg.__path__ = [str(_SRC_DIR)]
_pkg.__package__ = "robot_control"
sys.modules["robot_control"] = _pkg
