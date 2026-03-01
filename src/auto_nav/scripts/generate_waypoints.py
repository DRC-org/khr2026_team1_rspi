#!/usr/bin/env python3
"""PGM マップから壁を検出し、絶対座標の waypoints.yaml を生成するスタンドアロンスクリプト。

使い方:
    python3 generate_waypoints.py \
        --map /home/taiga/maps/field.pgm \
        --meta /home/taiga/maps/field.yaml \
        --relative src/auto_nav/config/waypoints_relative.yaml \
        --output src/auto_nav/config/waypoints.yaml

    # 確認のみ（ファイルを書き出さない）
    python3 generate_waypoints.py ... --dry-run
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path
from statistics import median

import yaml


# ---------------------------------------------------------------------------
# PGM 読み込み（Pillow 不要）
# ---------------------------------------------------------------------------

def _skip_comments(lines: list[str], idx: int) -> int:
    """コメント行（# で始まる行）をスキップして次のインデックスを返す。"""
    while idx < len(lines) and lines[idx].strip().startswith("#"):
        idx += 1
    return idx


def _read_pgm_tokens(data: bytes) -> tuple[str, int, int, int, list[int]]:
    """PGM バイナリ全体をパースして (magic, width, height, maxval, pixels) を返す。

    P2 (ASCII) と P5 (バイナリ) の両形式に対応。
    """
    text = data.decode("latin-1")
    lines = text.split("\n")

    idx = 0
    idx = _skip_comments(lines, idx)
    magic = lines[idx].strip()
    idx += 1

    # width height
    idx = _skip_comments(lines, idx)
    wh_parts: list[str] = []
    while len(wh_parts) < 2:
        idx = _skip_comments(lines, idx)
        wh_parts += lines[idx].strip().split()
        idx += 1
    width, height = int(wh_parts[0]), int(wh_parts[1])

    # maxval
    idx = _skip_comments(lines, idx)
    maxval = int(lines[idx].strip())
    idx += 1

    if magic == "P2":
        # ASCII: 残り全テキストをトークン分割
        remaining = " ".join(lines[idx:])
        pixels = [int(v) for v in remaining.split()]
    elif magic == "P5":
        # バイナリ: ヘッダ終端（maxval 行末の \n）以降がピクセルデータ
        # latin-1 デコード前のバイナリオフセットを求める
        header_text = "\n".join(lines[:idx])
        header_bytes = header_text.encode("latin-1") + b"\n"
        raw = data[len(header_bytes):]
        if maxval < 256:
            pixels = list(raw[:width * height])
        else:
            # 16-bit big-endian
            pixels = [
                (raw[i * 2] << 8) | raw[i * 2 + 1]
                for i in range(width * height)
            ]
    else:
        raise ValueError(f"未対応の PGM フォーマット: {magic}")

    return magic, width, height, maxval, pixels


# ---------------------------------------------------------------------------
# 壁検出
# ---------------------------------------------------------------------------

def _detect_walls(
    pixels: list[int],
    width: int,
    height: int,
    maxval: int,
    occupied_thresh: float,
    negate: bool,
) -> tuple[float, float, float, float]:
    """PGM ピクセルデータから壁ピクセルを検出し、各壁の row/col の中央値を返す。

    Returns:
        (north_row, south_row, west_col, east_col): ピクセル座標での壁位置
    """
    # negate=0 の場合: 低輝度ピクセルが occupied (壁)
    # negate=1 の場合: 高輝度ピクセルが occupied
    threshold = (1.0 - occupied_thresh) * maxval

    def is_wall(pix: int) -> bool:
        if negate:
            return pix > threshold
        return pix < threshold

    def pixel(row: int, col: int) -> int:
        return pixels[row * width + col]

    # 各列の最小・最大 occupied row
    north_rows: list[int] = []  # 最小 row = 画像上端 = world Y 最大（north）
    south_rows: list[int] = []  # 最大 row = 画像下端 = world Y 最小（south）
    for col in range(width):
        occ_rows = [row for row in range(height) if is_wall(pixel(row, col))]
        if occ_rows:
            north_rows.append(min(occ_rows))
            south_rows.append(max(occ_rows))

    # 各行の最小・最大 occupied col
    west_cols: list[int] = []
    east_cols: list[int] = []
    for row in range(height):
        occ_cols = [col for col in range(width) if is_wall(pixel(row, col))]
        if occ_cols:
            west_cols.append(min(occ_cols))
            east_cols.append(max(occ_cols))

    if not north_rows or not west_cols:
        raise RuntimeError(
            "壁ピクセルが検出されませんでした。"
            "occupied_thresh の値か PGM ファイルを確認してください。"
        )

    return (
        float(median(north_rows)),
        float(median(south_rows)),
        float(median(west_cols)),
        float(median(east_cols)),
    )


# ---------------------------------------------------------------------------
# 座標変換
# ---------------------------------------------------------------------------

def _pixel_to_world(
    row: float,
    col: float,
    origin: list[float],
    resolution: float,
    height: int,
) -> tuple[float, float]:
    """ピクセル座標 → ワールド座標変換。

    変換式:
        x = x0 + col * resolution
        y = y0 + (height - 1 - row) * resolution
    """
    x0, y0 = origin[0], origin[1]
    x = x0 + col * resolution
    y = y0 + (height - 1 - row) * resolution
    return x, y


# ---------------------------------------------------------------------------
# ウェイポイント座標計算
# ---------------------------------------------------------------------------

def _compute_absolute(
    wp: dict,
    north_y: float,
    south_y: float,
    west_x: float,
    east_x: float,
) -> tuple[float, float]:
    """相対定義から絶対 (x, y) を計算する。"""
    from_wall = wp["from_wall"]
    distance = float(wp["distance"])
    cross_wall = wp["cross_wall"]
    cross_distance = float(wp["cross_distance"])

    if from_wall == "south":
        y = south_y + distance
    elif from_wall == "north":
        y = north_y - distance
    elif from_wall == "west":
        y = west_x + distance  # from_wall が east/west の場合は y ではなく x 扱い
    elif from_wall == "east":
        y = east_x - distance
    else:
        raise ValueError(f"不明な from_wall: {from_wall!r}")

    if cross_wall == "west":
        x = west_x + cross_distance
    elif cross_wall == "east":
        x = east_x - cross_distance
    elif cross_wall == "south":
        x = south_y + cross_distance
    elif cross_wall == "north":
        x = north_y - cross_distance
    else:
        raise ValueError(f"不明な cross_wall: {cross_wall!r}")

    # from_wall が east/west のときは x/y が入れ替わっている
    if from_wall in ("west", "east"):
        x, y = y, x

    return x, y


# ---------------------------------------------------------------------------
# メイン処理
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="PGM マップから壁を検出し waypoints.yaml を自動生成する"
    )
    parser.add_argument("--map", required=True, help="入力 PGM ファイルパス")
    parser.add_argument("--meta", required=True, help="入力 YAML メタデータファイルパス（map_saver_cli 生成）")
    parser.add_argument("--relative", required=True, help="壁基準ウェイポイント定義 YAML パス")
    parser.add_argument("--output", required=True, help="出力 waypoints.yaml パス")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="検出結果と生成ウェイポイントを表示するだけでファイルを書き出さない",
    )
    parser.add_argument(
        "--occupied-thresh",
        type=float,
        default=None,
        help="occupied 判定閾値の上書き（デフォルト: YAML の occupied_thresh を使用）",
    )
    args = parser.parse_args()

    # --- PGM 読み込み ---
    pgm_path = Path(args.map)
    if not pgm_path.exists():
        print(f"[ERROR] PGM ファイルが見つかりません: {pgm_path}", file=sys.stderr)
        sys.exit(1)

    print(f"PGM を読み込み中: {pgm_path}")
    raw = pgm_path.read_bytes()
    _magic, width, height, maxval, pixels = _read_pgm_tokens(raw)
    print(f"  サイズ: {width} x {height} px, maxval: {maxval}")

    # --- メタデータ読み込み ---
    meta_path = Path(args.meta)
    if not meta_path.exists():
        print(f"[ERROR] メタデータファイルが見つかりません: {meta_path}", file=sys.stderr)
        sys.exit(1)

    with meta_path.open() as f:
        meta = yaml.safe_load(f)

    resolution: float = meta["resolution"]
    origin: list[float] = meta["origin"]
    negate: bool = bool(meta.get("negate", 0))
    occupied_thresh: float = args.occupied_thresh or meta.get("occupied_thresh", 0.65)

    print(f"  解像度: {resolution} m/px, origin: {origin}")
    print(f"  occupied_thresh: {occupied_thresh}, negate: {negate}")

    # --- 壁検出 ---
    print("\n壁を検出中...")
    north_row, south_row, west_col, east_col = _detect_walls(
        pixels, width, height, maxval, occupied_thresh, negate
    )

    north_x, north_y = _pixel_to_world(north_row, west_col, origin, resolution, height)
    south_x, south_y = _pixel_to_world(south_row, west_col, origin, resolution, height)
    west_x, west_y = _pixel_to_world(north_row, west_col, origin, resolution, height)
    east_x, east_y = _pixel_to_world(north_row, east_col, origin, resolution, height)

    # north/south は Y 座標のみ、west/east は X 座標のみ使用
    _, north_y = _pixel_to_world(north_row, 0, origin, resolution, height)
    _, south_y = _pixel_to_world(south_row, 0, origin, resolution, height)
    west_x, _ = _pixel_to_world(0, west_col, origin, resolution, height)
    east_x, _ = _pixel_to_world(0, east_col, origin, resolution, height)

    print("\nDetected walls:")
    print(f"  north_y = {north_y:.4f} m  (row {north_row:.0f})")
    print(f"  south_y = {south_y:.4f} m  (row {south_row:.0f})")
    print(f"  west_x  = {west_x:.4f} m  (col {west_col:.0f})")
    print(f"  east_x  = {east_x:.4f} m  (col {east_col:.0f})")

    # --- 相対定義の読み込み ---
    relative_path = Path(args.relative)
    if not relative_path.exists():
        print(f"[ERROR] 相対定義ファイルが見つかりません: {relative_path}", file=sys.stderr)
        sys.exit(1)

    with relative_path.open() as f:
        relative_data = yaml.safe_load(f)

    auto_sequence: list[str] = relative_data.get("auto_sequence", [])
    relative_waypoints: dict = relative_data.get("waypoints", {})

    # --- 絶対座標計算 ---
    print("\nGenerated waypoints:")
    abs_waypoints: dict = {}
    for name, wp in relative_waypoints.items():
        x, y = _compute_absolute(wp, north_y, south_y, west_x, east_x)
        theta = float(wp.get("theta", 0.0))
        on_arrive = wp.get("on_arrive", [])
        abs_waypoints[name] = {"x": round(x, 4), "y": round(y, 4), "theta": theta, "on_arrive": on_arrive}
        print(f"  {name}: x={x:.4f}, y={y:.4f}, theta={theta}")

    if args.dry_run:
        print("\n[DRY RUN] ファイルへの書き出しをスキップしました。")
        return

    # --- 出力 YAML 生成 ---
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    map_name = pgm_path.stem
    header = (
        f"# 自動生成: generate_waypoints.py により {now} に生成\n"
        f"# 入力: {map_name}.pgm"
        f" (検出壁: north={north_y:.4f}, south={south_y:.4f},"
        f" west={west_x:.4f}, east={east_x:.4f})\n"
    )

    output_data: dict = {}
    if auto_sequence:
        output_data["auto_sequence"] = auto_sequence
    output_data["waypoints"] = abs_waypoints

    with output_path.open("w") as f:
        f.write(header)
        yaml.dump(output_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    print(f"\n出力完了: {output_path}")


if __name__ == "__main__":
    main()
