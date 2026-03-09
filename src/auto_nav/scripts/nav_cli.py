#!/usr/bin/env python3
"""
nav_cli.py — routing_node を bluetooth_rx 経由でコマンド操作する CLI ツール

【一発コマンド】
  ros2 run auto_nav nav_cli.py -- mode auto
  ros2 run auto_nav nav_cli.py -- mode manual
  ros2 run auto_nav nav_cli.py -- court blue
  ros2 run auto_nav nav_cli.py -- court red
  ros2 run auto_nav nav_cli.py -- start
  ros2 run auto_nav nav_cli.py -- start 2
  ros2 run auto_nav nav_cli.py -- stop
  ros2 run auto_nav nav_cli.py -- go <waypoint>

【インタラクティブモード】
  ros2 run auto_nav nav_cli.py
"""

import json
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

HELP = """\
コマンド一覧:
  mode auto/manual        モード切り替え
  court blue/red          コート設定
  start [N]               自動シーケンス開始（N: 開始インデックス、省略=0）
  stop                    自動シーケンス停止
  go <waypoint>           ウェイポイントへ移動
  help                    このヘルプを表示
  quit / exit / Ctrl-D    終了
"""


def parse_args_to_payload(args: list[str]) -> dict | None:
    if not args:
        return None
    cmd = args[0].lower()
    match cmd:
        case "mode" if len(args) == 2 and args[1] in ("auto", "manual"):
            return {"type": "nav_mode", "mode": args[1]}
        case "court" if len(args) == 2 and args[1] in ("blue", "red"):
            return {"type": "set_court", "court": args[1]}
        case "start":
            try:
                idx = int(args[1]) if len(args) >= 2 else 0
            except ValueError:
                return None
            return {"type": "start_auto", "from_index": idx}
        case "stop":
            return {"type": "stop_auto"}
        case "go" if len(args) == 2:
            return {"type": "nav_goal", "waypoint": args[1]}
    return None


def _print_status(data: dict) -> None:
    nav = data.get("nav_status")
    if nav == "mode":
        print(f"[rspi] mode → {data.get('mode')}")
    elif nav == "navigating":
        idx = data.get("seq_index")
        total = data.get("seq_total")
        prog = f" ({idx + 1}/{total})" if idx is not None and total is not None else ""
        print(f"[rspi] navigating → {data.get('waypoint', '?')}{prog}")
    elif nav == "arrived":
        idx = data.get("seq_index")
        total = data.get("seq_total")
        prog = f" ({idx + 1}/{total})" if idx is not None and total is not None else ""
        print(f"[rspi] arrived {data.get('waypoint', '?')}{prog}")
    elif nav == "completed":
        print("[rspi] auto sequence completed")
    elif nav == "cancelled":
        print("[rspi] cancelled")
    elif nav == "error":
        msg = data.get("message", "")
        wp = data.get("waypoint", "-")
        idx = data.get("seq_index")
        total = data.get("seq_total")
        prog = f" ({idx + 1}/{total})" if idx is not None and total is not None else ""
        print(f"[rspi] ERROR: {msg} (waypoint={wp}){prog}")
    elif nav == "court_set":
        print(f"[rspi] court → {data.get('court')}")
    elif nav == "relocating":
        cd = data.get("countdown")
        wp = data.get("waypoint", "-")
        print(f"[rspi] relocating... {cd}秒 → {wp}")
    else:
        print(f"[rspi] {json.dumps(data, ensure_ascii=False)}")


class NavCliNode(Node):
    def __init__(self):
        super().__init__("nav_cli")
        self._pub = self.create_publisher(String, "bluetooth_rx", 10)
        self._sub = self.create_subscription(String, "bluetooth_tx", self._on_tx, 10)
        self._response_event = threading.Event()

    def send(self, payload: dict) -> None:
        self._response_event.clear()
        self._pub.publish(String(data=json.dumps(payload)))

    def wait_response(self, timeout: float = 3.0) -> None:
        self._response_event.wait(timeout=timeout)

    def _on_tx(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # robot_pos は高頻度で来るため無視
        if data.get("type") == "robot_pos":
            return
        self._response_event.set()
        _print_status(data)


def run_oneshot(node: NavCliNode, args: list[str]) -> int:
    payload = parse_args_to_payload(args)
    if payload is None:
        print(f"不明なコマンド: {' '.join(args)}", file=sys.stderr)
        print(HELP, file=sys.stderr)
        return 1
    node.send(payload)
    node.wait_response(timeout=3.0)
    return 0


def run_interactive(node: NavCliNode) -> None:
    print("nav_cli インタラクティブモード（Ctrl-D または quit で終了）")
    print(HELP)
    try:
        while True:
            try:
                line = input("nav> ").strip()
            except EOFError:
                print()
                break
            if not line:
                continue
            args = line.split()
            if args[0].lower() in ("quit", "exit"):
                break
            if args[0].lower() == "help":
                print(HELP)
                continue
            payload = parse_args_to_payload(args)
            if payload is None:
                print(f"不明なコマンド: {line}")
                print(HELP)
                continue
            node.send(payload)
    except KeyboardInterrupt:
        print()


def main():
    rclpy.init()
    node = NavCliNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    cli_args = sys.argv[1:]
    exit_code = 0
    try:
        if cli_args:
            exit_code = run_oneshot(node, cli_args)
        else:
            run_interactive(node)
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
