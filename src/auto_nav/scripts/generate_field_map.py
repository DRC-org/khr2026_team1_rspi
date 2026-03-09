#!/usr/bin/env python3
"""ルールブック寸法から合成フィールドマップ（PGM + YAML）を生成するスクリプト。

使い方:
    ros2 run auto_nav generate_field_map.py
    ros2 run auto_nav generate_field_map.py -- --output-dir /path/to/maps
    python3 generate_field_map.py
    python3 generate_field_map.py --output-dir /path/to/maps

出力（デフォルト）:
    /home/pi/maps/field_synthetic.pgm
    /home/pi/maps/field_synthetic.yaml

入力設定:
    config/field_dimensions.yaml（本パッケージ内）
"""

import argparse
import os
import struct
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def _load_field_dimensions() -> dict:
    pkg_dir = get_package_share_directory("auto_nav")
    config_path = os.path.join(pkg_dir, "config", "field_dimensions.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def _generate_pgm(
    width_m: float,
    height_m: float,
    wall_thickness_m: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    internal_walls: list,
    output_path: Path,
) -> None:
    """P5 バイナリ PGM を生成する。

    画素値:
        205 = free（nav2_map_server の map_saver_cli 標準値）
        0   = occupied（壁）
    """
    img_w = round(width_m / resolution)
    img_h = round(height_m / resolution)
    wall_px = max(1, round(wall_thickness_m / resolution))

    # free で初期化
    pixels = bytearray([205] * (img_w * img_h))

    def set_rect(row_start: int, row_end: int, col_start: int, col_end: int) -> None:
        for row in range(max(0, row_start), min(img_h, row_end)):
            for col in range(max(0, col_start), min(img_w, col_end)):
                pixels[row * img_w + col] = 0

    # 北壁（画像上端 = ROS world Y 最大 = north）
    set_rect(0, wall_px, 0, img_w)
    # 南壁（画像下端 = ROS world Y 最小 = south）
    set_rect(img_h - wall_px, img_h, 0, img_w)
    # 西壁（画像左端 = ROS world X 最小 = west）
    set_rect(0, img_h, 0, wall_px)
    # 東壁（画像右端 = ROS world X 最大 = east）
    set_rect(0, img_h, img_w - wall_px, img_w)

    # 内部壁（軸平行線分をバウンディングボックスで描画）
    for wall in internal_walls:
        x1, y1 = wall["x1"], wall["y1"]
        x2, y2 = wall["x2"], wall["y2"]
        half_t = wall["thickness"] / 2.0
        wx_min = min(x1, x2) - half_t
        wx_max = max(x1, x2) + half_t
        wy_min = min(y1, y2) - half_t
        wy_max = max(y1, y2) + half_t
        col_s = round((wx_min - origin_x) / resolution)
        col_e = round((wx_max - origin_x) / resolution) + 1
        row_s = img_h - 1 - round((wy_max - origin_y) / resolution)
        row_e = img_h - 1 - round((wy_min - origin_y) / resolution) + 1
        set_rect(row_s, row_e, col_s, col_e)
        print(f"  内部壁 '{wall['name']}' 描画: ({x1},{y1})→({x2},{y2}) 厚み {wall['thickness']}m")

    header = f"P5\n{img_w} {img_h}\n255\n".encode("ascii")
    output_path.write_bytes(header + bytes(pixels))
    print(f"  PGM 生成完了: {output_path}  ({img_w} x {img_h} px)")


def _generate_yaml(
    pgm_filename: str,
    origin_x: float,
    origin_y: float,
    resolution: float,
    output_path: Path,
) -> None:
    meta = {
        "image": pgm_filename,
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with output_path.open("w") as f:
        yaml.dump(meta, f, default_flow_style=False)
    print(f"  YAML 生成完了: {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="合成フィールドマップ（PGM+YAML）を生成")
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/home/pi/maps",
        help="出力ディレクトリ（デフォルト: /home/pi/maps）",
    )
    args = parser.parse_args()

    config = _load_field_dimensions()

    field = config["field"]
    origin = config["origin"]
    resolution: float = config["resolution"]

    width_m: float = field["width"]
    height_m: float = field["height"]
    wall_thickness_m: float = field["wall_thickness"]
    origin_x: float = origin["x"]
    origin_y: float = origin["y"]

    print("フィールド寸法:")
    print(f"  幅(X): {width_m} m, 高さ(Y): {height_m} m")
    print(f"  壁厚: {wall_thickness_m} m, 解像度: {resolution} m/cell")
    print(f"  マップ原点: ({origin_x}, {origin_y})")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    pgm_path = output_dir / "field_synthetic.pgm"
    yaml_path = output_dir / "field_synthetic.yaml"

    internal_walls: list = config.get("internal_walls", [])

    print("\nマップ生成中...")
    _generate_pgm(
        width_m, height_m, wall_thickness_m, resolution,
        origin_x, origin_y, internal_walls, pgm_path,
    )
    _generate_yaml("field_synthetic.pgm", origin_x, origin_y, resolution, yaml_path)

    print("\n完了。config/waypoints.yaml を直接編集してウェイポイントを設定してください。")


if __name__ == "__main__":
    main()
