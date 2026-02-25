# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 作業ルール（必ず守ること）

### デバッグ・動作確認のワークフロー

**基本方針**: Claude がコマンドを打って直接デバッグできる環境を活用する。

- ノード起動が必要なときは、ユーザーに起動を依頼する（**起動コマンドを必ず明記すること**）
- ユーザーがノードを起動したら、Claude が自由に診断コマンドを実行してデバッグを進める
- ロボット実機の物理的な動作確認（走行・センサー確認など）は引き続きユーザーが行う

**ユーザーへの起動依頼の書き方（例）**:
```
以下のコマンドでノードを起動してください:
  ros2 launch auto_nav mapping_launch.py
起動したら教えてください。デバッグを進めます。
```

### ROS2 診断コマンドの注意事項
- `ros2 topic list` / `ros2 node list` はデーモン経由のため slam_toolbox 等が見えないことがある
  → `--no-daemon` オプションを使うか、`ros2 service call` で直接確認する
- slam_toolbox の lifecycle 確認: `ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState`
- `/map` 等の確認: `ros2 topic echo --qos-reliability reliable /map nav_msgs/msg/OccupancyGrid --field info`

### 実装完了時の記録
実装完了後は、動作確認に必要なコマンドをすべてこのチャットに提示すること。
また、その確認手順を `auto_routing_plan.md` の該当フェーズに記録すること。

### 作業記録の更新
作業内容・実装上の変更点・気づいた問題や注意事項は、作業中・完了時を問わず、
漏れなく `auto_routing_plan.md` の「作業記録」セクションに追記すること。

---

## 概要

関西春ロボコン 2026 チーム 1 の Raspberry Pi 側実装。
**ROS 2 Jazzy Jalisco** を使用。4 輪メカナムホイール（M3508 モータ）＋ YDLiDAR による自律走行システム。

詳細な実装計画・作業記録は `auto_routing_plan.md` を参照。

### リポジトリ構成

| リポジトリ | 説明 |
|-----------|------|
| `khr2026_team1_rspi`（本リポジトリ） | Raspberry Pi 側の中央制御プログラム（ROS2） |
| `khr2026_team1_cwmc` | 足回りモータコントローラ（ESP32）。本リポジトリからのコマンドを受け取り M3508 を駆動 |
| `khr2026_team1_hwmc` | ロボットハンド等モータコントローラ（ESP32）。本リポジトリからのコマンドを受け取り手先モータを駆動 |

---

## ビルドコマンド

### 通常パッケージ（Python）

```bash
colcon build --packages-select <pkg> --symlink-install
source install/setup.bash
```

`--symlink-install` によりビルドなしでソース変更が反映される。

### micro_ros_agent（初回のみ）

```bash
colcon build --packages-up-to micro_ros_agent \
    --cmake-args \
    "-DUAGENT_BUILD_EXECUTABLE=OFF" \
    "-DUAGENT_P2P_PROFILE=OFF" \
    "--no-warn-unused-cli"
```

### 新規 Python パッケージの作成手順

```bash
cd src
ros2 pkg create <pkg_name>
cd <pkg_name>
uv init . --lib --python-preference only-system
uv venv --system-site-packages
```

---

## 起動コマンド

### 全ノード一括起動（マッピング用・通常使用）

```bash
# シリアル接続（デフォルト, USB）
ros2 launch auto_nav mapping_launch.py
# ESP32 が /dev/ttyUSB1 の場合:
ros2 launch auto_nav mapping_launch.py serial_port:=/dev/ttyUSB1
# Wi-Fi 接続（UDP, port 8888）
ros2 launch auto_nav mapping_launch.py transport:=udp4
```

### bt_communication 単独実行時

```bash
cd src/bt_communication
source .venv/bin/activate
ros2 run bt_communication launch.py
```

---

## よく使うデバッグコマンド

```bash
# TF ツリーを確認（odom → base_link が存在するか）
ros2 run tf2_tools view_frames

# トピック確認
ros2 topic hz /odom
ros2 topic echo wheel_feedback
ros2 topic echo /scan

# nav_mode を手動で切り替え
ros2 topic pub /nav_mode std_msgs/String "data: 'auto'" --once
ros2 topic pub /nav_mode std_msgs/String "data: 'manual'" --once

# cmd_vel テスト（auto モード時）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once

# SLAM 地図保存（slam_toolbox ネイティブ形式 .posegraph + .data）
# ※ save_map は nav2_map_saver が必要で失敗する。serialize_map を使うこと
mkdir -p /home/taiga/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "filename: '/home/taiga/maps/field'"
```

---

## アーキテクチャ

### ノード間のデータフロー

```
[Bluetooth Controller]
        ↓ bluetooth_rx (JSON, std_msgs/String)
[bt_communication] ←→ [robot_control (RobotController)]
                              ↓ wheel_control (WheelMessage)  ← manual モード時
[ESP32 (micro_ros_agent)] ←→ wheel_feedback (WheelMessage, ~20Hz)
                              ↑ hand_control / hand_feedback
[auto_nav OdometryNode]   ← wheel_feedback → /odom + TF(odom→base_link)
[auto_nav CmdVelBridgeNode] ← /cmd_vel → wheel_control  ← auto モード時
[ydlidar_ros2_driver]     → /scan (LaserScan, 10Hz)
```

### モード切り替え（`/nav_mode`）

`/nav_mode` トピック（`std_msgs/String`）で "manual" / "auto" を切り替える。

| モード | wheel_control の担当 | robot_control の動作 |
|--------|---------------------|---------------------|
| manual | robot_control（Bluetooth → 車輪） | `send_control_command()` 実行 |
| auto | cmd_vel_bridge（/cmd_vel → 車輪） | `send_control_command()` をスキップ |

切り替え時の競合防止:
- `auto → manual` 時に cmd_vel_bridge がゼロ RPM を送信
- `manual → auto` 時に robot_control がジョイスティック入力をリセット

### TF ツリー

```
map
 └── odom              ← slam_toolbox が配信（フェーズ3以降）
      └── base_link    ← OdometryNode が配信
           └── laser_frame  ← static_tf で定義（LiDAR 取付位置）
```

---

## パッケージ構成

| パッケージ | 説明 |
|-----------|------|
| `robot_msgs` | カスタムメッセージ定義（WheelMessage, HandMessage 等） |
| `robot_control` | Bluetooth 入力 → ESP32 制御（M3508 逆運動学含む） |
| `bt_communication` | Bluetooth GATT サーバー（bumble 依存、`.venv` で管理） |
| `auto_nav` | 自律走行（odometry_node, cmd_vel_bridge_node、以降追加予定） |
| `ydlidar_ros2_driver` | YDLiDAR ドライバ（/scan 配信） |
| `micro_ros_agent` | ESP32 ↔ ROS2 ブリッジ |
| `slam_toolbox` | SLAM（サブモジュール、フェーズ3で使用予定） |

---

## 運動学（重要）

物理定数は `src/robot_control/src/robot_control/constants.py` が正式定義。

```
WHEEL_RADIUS = 0.04925 m
GEAR_RATIO   = 19.20320855614973
L_X = 0.1725 m  (前後方向: ロボット中心 → タイヤ中心)
L_Y = 0.2425 m  (左右方向: ロボット中心 → タイヤ中心)
G   = L_X + L_Y = 0.415 m
```

**逆運動学**（`m3508.py` / `cmd_vel_bridge_node.py`）:
```
v_fl = +Vx + Vy - G*ω
v_fr = -Vx + Vy - G*ω   ← FR/RR は取付向きが逆のため符号反転済み
v_rl = +Vx - Vy - G*ω
v_rr = -Vx - Vy - G*ω
```

**前進運動学**（`odometry_node.py`）:
```
Vx    = ( v_fl - v_fr + v_rl - v_rr) / 4
Vy    = ( v_fl + v_fr - v_rl - v_rr) / 4
omega = -(v_fl + v_fr + v_rl + v_rr) / (4 * G)
```

FR/RR の wheel_feedback RPM は取付向きの符号反転がそのまま返ってくるため、変換式に追加の符号処理は不要。

---

## bumble（bt_communication）の注意事項

bumble は `src/bt_communication/.venv` にのみインストール済み。
`mapping_launch.py` では bt_communication ノードにのみ `PYTHONPATH` を設定して対応している。
システム全体の `PYTHONPATH` を変更しないこと（他パッケージが壊れる）。

---

## 必要パッケージ（フェーズ3以降）

```bash
sudo apt install \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-dwb-core \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-nav2-map-server \
  ros-jazzy-laser-filters
```
