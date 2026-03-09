#!/usr/bin/env python3
"""
nav2_params.yaml のパラメータを実行中の Nav2 ノードにホットリロードするスクリプト。

使い方:
  ros2 run auto_nav reload_nav2_params.py                  # 1回だけリロード
  ros2 run auto_nav reload_nav2_params.py --watch          # ファイル変更を監視して自動リロード
  ros2 run auto_nav reload_nav2_params.py --yaml /path/to/params.yaml
"""

import argparse
import os
import subprocess
import time

import yaml

_DEFAULT_YAML = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "../config/nav2_params.yaml")
)


def _load_node_names(yaml_path: str) -> list[str]:
    """YAML のトップレベルキーからノード名を取得する（コメント行・無効キーは除外）。"""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return []
    nodes = []
    for key, value in data.items():
        if not isinstance(value, dict):
            continue
        if "ros__parameters" not in value:
            continue
        nodes.append(key)
    return nodes


def _reload_once(yaml_path: str) -> None:
    yaml_path = os.path.abspath(yaml_path)
    nodes = _load_node_names(yaml_path)
    if not nodes:
        print(f"[warn] {yaml_path} にリロード対象ノードが見つかりませんでした")
        return

    print(f"Reloading {yaml_path} ...")
    for node in nodes:
        result = subprocess.run(
            ["ros2", "param", "load", f"/{node}", yaml_path],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            print(f"  \u2713 {node}")
        else:
            # ノードが起動していない場合などはスキップ扱いにする
            err = (result.stderr or result.stdout).strip().split("\n")[-1]
            print(f"  \u2717 {node}: {err}")


def _watch(yaml_path: str) -> None:
    print(f"Watching {yaml_path} (Ctrl+C で停止) ...")
    mtime: float | None = None
    while True:
        try:
            new_mtime = os.path.getmtime(yaml_path)
            if mtime is not None and new_mtime != mtime:
                _reload_once(yaml_path)
            mtime = new_mtime
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopped.")
            break


def main() -> None:
    parser = argparse.ArgumentParser(
        description="nav2_params.yaml を実行中のノードにホットリロードする"
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="ファイル変更を監視して自動リロード",
    )
    parser.add_argument(
        "--yaml",
        default=_DEFAULT_YAML,
        help=f"対象 YAML ファイルパス（デフォルト: {_DEFAULT_YAML}）",
    )
    args = parser.parse_args()

    if args.watch:
        _watch(args.yaml)
    else:
        _reload_once(args.yaml)


if __name__ == "__main__":
    main()
