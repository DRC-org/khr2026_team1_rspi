#!/usr/bin/env python3
"""field_dimensions.yaml から Gazebo 用フィールド壁 SDF を生成する。

マップ（PGM）の黒い壁と Gazebo の壁を同一定義で一致させる。
使い方:
  ros2 run khr2026_team1_simulation generate_field_sdf.py
  ros2 run khr2026_team1_simulation generate_field_sdf.py -- --config /path/to/field_dimensions.yaml
  python3 generate_field_sdf.py --output /path/to/khr2026_field.sdf
"""

import argparse
import math
import os
import sys
from pathlib import Path

import yaml


def _load_field_dimensions(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def _wall_box_sdf(name: str, x: float, y: float, z: float, size_x: float, size_y: float, size_z: float, yaw: float = 0.0) -> str:
    """1本の壁を表す SDF model を返す。"""
    return f"""    <!-- {name} -->
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def _generate_walls_sdf(config: dict) -> str:
    field = config["field"]
    origin = config["origin"]
    ox = origin["x"]
    oy = origin["y"]
    W = field["width"]
    H = field["height"]
    t = field["wall_thickness"]
    z = 0.25
    wall_height = 0.5

    # RViz のマップ表示に合わせる: map_server は画像行0を origin.y に置く。
    # generate_field_map は行0=北(Y最大)で描くため、マップは北がy=0・南がy=Hに見える。
    # Gazebo を同じ向きにするため Y を反転: y_gz = (oy + H) - y_world
    def flip_y(y_world: float) -> float:
        return (oy + H) - y_world

    lines = []

    # 外周壁（南・北・西・東）— Y 反転で北が y=0 側に
    lines.append(_wall_box_sdf("wall_south", ox + W / 2, flip_y(oy + t / 2), z, W, t, wall_height))
    lines.append(_wall_box_sdf("wall_north", ox + W / 2, flip_y(oy + H - t / 2), z, W, t, wall_height))
    lines.append(_wall_box_sdf("wall_west", ox + t / 2, flip_y(oy + H / 2), z, t, H, wall_height))
    lines.append(_wall_box_sdf("wall_east", ox + W - t / 2, flip_y(oy + H / 2), z, t, H, wall_height))

    # 内部壁（Y 反転 + yaw 反転で向きを維持）
    for i, wall in enumerate(config.get("internal_walls", [])):
        x1, y1 = wall["x1"], wall["y1"]
        x2, y2 = wall["x2"], wall["y2"]
        th = wall["thickness"]
        name = wall.get("name", f"internal_wall_{i}").replace(" ", "_")
        L = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if L < 1e-6:
            continue
        cx = (x1 + x2) / 2
        cy = flip_y((y1 + y2) / 2)
        yaw = math.atan2(y2 - y1, x2 - x1)
        yaw_flip = -yaw  # Y 反転で線分の向きが反転するため
        lines.append(_wall_box_sdf(name, cx, cy, z, L, th, wall_height, yaw_flip))

    return "\n".join(lines)


def _full_world_sdf(walls_xml: str) -> str:
    """world 全体の SDF（light, ground_plane, plugins + 壁）を返す。"""
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="khr2026_field_generated">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

{walls_xml}
  </world>
</sdf>
'''


def main() -> None:
    parser = argparse.ArgumentParser(description="field_dimensions.yaml から Gazebo 用 SDF を生成")
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="field_dimensions.yaml のパス（未指定時は auto_nav の share から取得）",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="出力 SDF ファイルパス（未指定時は本パッケージの worlds/khr2026_field.sdf）",
    )
    args = parser.parse_args()

    if args.config:
        config_path = Path(args.config)
        if not config_path.is_file():
            print(f"Error: config not found: {config_path}", file=sys.stderr)
            sys.exit(1)
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            auto_nav_share = get_package_share_directory("auto_nav")
            config_path = Path(auto_nav_share) / "config" / "field_dimensions.yaml"
            if not config_path.is_file():
                print(f"Error: auto_nav config not found: {config_path}", file=sys.stderr)
                sys.exit(1)
        except Exception as e:
            print(f"Error: --config を指定するか、auto_nav をビルドしてから実行してください: {e}", file=sys.stderr)
            sys.exit(1)

    config = _load_field_dimensions(str(config_path))
    walls_xml = _generate_walls_sdf(config)
    sdf_content = _full_world_sdf(walls_xml)

    if args.output:
        out_path = Path(args.output)
    else:
        # 本スクリプトの場所から worlds/khr2026_field.sdf を相対で決める
        script_dir = Path(__file__).resolve().parent
        pkg_dir = script_dir.parent
        out_path = pkg_dir / "worlds" / "khr2026_field.sdf"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(sdf_content, encoding="utf-8")
    print(f"Generated: {out_path}")


if __name__ == "__main__":
    main()
