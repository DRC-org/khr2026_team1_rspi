# ROS2 + LiDAR 自律走行 実装計画

## 1. 概要・目標アーキテクチャ

4輪メカナムホイールロボット（M3508 + YDLiDAR）に自律走行機能を追加する。
試合前にルールブック寸法から合成 PGM マップを生成し、AMCL で自己位置推定して Nav2 で走行する。

```
[目標位置指定]
  └── Bluetooth / WebSocket (JSON: {"type": "nav_goal", "waypoint": "goal_A"})
              ↓
[routing_node]  ←── /map, AMCL 自己位置
  ├── Nav2 ActionClient (NavigateToPose)
  └── /cmd_vel ──→ [cmd_vel_bridge_node] ──→ wheel_control ──→ [ESP32]

[map_server]                  ──→ /map（合成 PGM）
[amcl]                        ←── /scan_filtered, /odom ──→ map→odom TF
[odometry_node]               ←── wheel_feedback ──→ /odom_raw
[imu_publisher_node]          ←── wheel_feedback ──→ /imu（起動時バイアスキャリブ後に配信）
[ekf_filter_node]             ←── /odom_raw, /imu ──→ /odom, TF(odom→base_link)
[YDLiDAR]                     ──→ /scan → [laser_filter] ──→ /scan_filtered
[web_server_node]             ←→ bluetooth_rx/bluetooth_tx ←→ WebSocket (port 8080)
```

### 競技運用フロー

```
① 合成マップ生成（フィールド寸法変更時のみ）
   ros2 run auto_nav generate_field_map.py
   → /home/pi/maps/field_synthetic.pgm + field_synthetic.yaml が生成される

② 自律走行起動
   ros2 launch auto_nav auto_nav_launch.py
   （マップを明示指定する場合: map:=/home/pi/maps/field_synthetic.yaml）

③ コート選択 → Web UI でコート（青/赤）を選択
   → routing_node が AMCL に /initialpose を送信して自己位置を初期化

④ 自動制御開始 → Web UI の「自動開始」ボタン → auto_sequence を実行

⑤ フォールバック（自動制御不能時）
   iPad を Bluetooth 接続 → 手動操縦モードに切り替え
```

---

## 2. 現在のシステム構成

### 2-1. ノード一覧（auto_nav_launch.py 起動時）

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `micro_ros_agent_c` | micro_ros_agent | cwmc ESP32 ↔ ROS2 ブリッジ（/dev/esp32_c or UDP:8888） |
| `micro_ros_agent_h` | micro_ros_agent | hwmc ESP32 ↔ ROS2 ブリッジ（/dev/esp32_h or UDP:8889） |
| ydlidar_ros2_driver | ydlidar_ros2_driver | LiDAR /scan 配信 |
| `static_tf_base_to_laser` | tf2_ros | base_link → laser_frame（yaw=1.5708, z=0.15m） |
| `scan_to_scan_filter_chain` | laser_filters | /scan → /scan_filtered（0.33m未満除去） |
| `odometry_node` | auto_nav | wheel_feedback → /odom_raw |
| `imu_publisher_node` | auto_nav | wheel_feedback → /imu（LSM9DS1） |
| `ekf_filter_node` | robot_localization | /odom_raw + /imu → /odom + TF(odom→base_link) |
| `map_server` | nav2_map_server | 合成 PGM → /map（lifecycle） |
| `amcl` | nav2_amcl | パーティクルフィルタ自己位置推定 → map→odom TF（lifecycle） |
| `lifecycle_manager_localization` | nav2_lifecycle_manager | map_server + amcl のライフサイクル管理（8秒遅延起動） |
| `controller_server` | nav2_controller | MPPI コントローラー（Omni モデル） |
| `planner_server` | nav2_planner | NavfnPlanner（A*） |
| `behavior_server` | nav2_behaviors | spin / backup / wait |
| `bt_navigator` | nav2_bt_navigator | BT ツリーで NavigateToPose を管理 |
| `lifecycle_manager_navigation` | nav2_lifecycle_manager | Nav2 ナビゲーションノードのライフサイクル管理 |
| `routing_node` | auto_nav | Bluetooth/WS コマンド → NavigateToPose ActionClient |
| `robot_control_node` | robot_control | Bluetooth 手動操縦 + ESP32 制御 |
| `bt_communication_node` | bt_communication | Bluetooth GATT サーバー（bumble、.venv 分離） |
| `web_server_node` | web_control | WebSocket ブリッジ（aiohttp、port 8080） |
| `cmd_vel_bridge_node` | auto_nav | /cmd_vel → wheel_control（auto モード時のみ） |

**lifecycle_manager_localization は EKF が安定するまで 8 秒遅延起動する**（TF なしで AMCL が起動するとスキャンキューが溢れて LiDAR を一切処理できなくなるため）。

### 2-2. TF ツリー

```
map
 └── odom              ← AMCL が配信（set_court コマンド後に確立）
      └── base_link    ← ekf_filter_node が配信（エンコーダ + IMU 融合）
           └── laser_frame  ← static_tf（yaw=1.5708, pitch=0.0, roll=0.0, z=0.15m）
```

`odometry_node` は `/odom_raw` のみ配信。`odom→base_link` TF は EKF が担当。
set_court コマンドを受信するまで AMCL は `/initialpose` を受け取れず、`map→odom` TF を publish しない（正常動作）。

### 2-3. 主要ファイル一覧

| ファイル | 説明 |
|---------|------|
| `src/auto_nav/launch/auto_nav_launch.py` | 全ノード一括起動（AMCL ローカリゼーションモード） |
| `src/auto_nav/src/auto_nav/odometry_node.py` | wheel_feedback → /odom_raw |
| `src/auto_nav/src/auto_nav/imu_publisher_node.py` | wheel_feedback → /imu（起動時バイアスキャリブ） |
| `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py` | /cmd_vel → wheel_control（ヨー PID 付き） |
| `src/auto_nav/src/auto_nav/routing_node.py` | Bluetooth/WS → NavigateToPose、on_arrive、set_court |
| `src/auto_nav/src/auto_nav/yagura_position_node.py` | /scan から前方±60°のΦ114mm円柱を最大3本検出、距離近い順に yagura_position_0/1/2 で配信 |
| `src/auto_nav/scripts/generate_field_map.py` | field_dimensions.yaml → 合成 PGM/YAML 生成 |
| `src/auto_nav/config/field_dimensions.yaml` | フィールド寸法・コート別初期位置 |
| `src/auto_nav/config/waypoints.yaml` | ウェイポイント絶対座標（直接編集） |
| `src/auto_nav/config/nav2_params.yaml` | Nav2 全パラメータ（MPPI + NavfnPlanner + AMCL lifecycle） |
| `src/auto_nav/config/amcl_params.yaml` | AMCL パラメータ（OmniMotionModel） |
| `src/auto_nav/config/ekf_params.yaml` | EKF パラメータ（robot_localization） |
| `src/auto_nav/config/slam_mapping_params.yaml` | SLAM マッピング用パラメータ（参照用・現在は不使用） |
| `src/auto_nav/config/slam_localization_params.yaml` | SLAM ローカリゼーション用パラメータ（参照用・現在は不使用） |
| `src/web_control/src/web_control/web_server_node.py` | WebSocket ブリッジノード（aiohttp） |
| `src/robot_control/src/robot_control/constants.py` | 運動学定数（正式定義） |

### 2-4. 起動コマンド

```bash
# 合成マップ生成（初回 + フィールド寸法変更時）
ros2 run auto_nav generate_field_map.py

# 自律走行起動（デフォルト: serial, マップ: field_synthetic.yaml）
ros2 launch auto_nav auto_nav_launch.py

# マップを明示指定
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field_synthetic.yaml

# WiFi UDP 接続
ros2 launch auto_nav auto_nav_launch.py transport:=udp4

# デバッグログ有効化
ros2 launch auto_nav auto_nav_launch.py debug:=true
```

---

## 3. フィールド・座標系・ウェイポイント

### 3-1. フィールド座標系（確定）

- 原点 (0,0): フィールド南西端外壁外側の交点
- +X = 東（青コート側）、+Y = 北、フィールド中心軸は X=0
- 赤コート = 左（-X）、青コート = 右（+X）。北南線（X=0）で線対称
- 赤ロボット: 左下スタート、北（+Y）向き（`yaw: 1.5708`）
- 青ロボット: 右下スタート、南（-Y）向き（`yaw: -1.5708`）

### 3-2. コート変換（routing_node.py `_apply_court_transform`）

- 青（デフォルト）: そのまま `(x, y, θ)`
- 赤: X反転 `(-x, y, π-θ)` ← **縦線（北南線）対称が正しい。旧実装の `(x, -y, -θ)` はバグだったため 2026-03-03 に修正済み**

### 3-3. フィールド寸法（field_dimensions.yaml 記載値）

| 項目 | 値 |
|------|-----|
| 全幅（東西、外寸） | 6.962 m |
| 全高（南北、外寸） | 7.0 m |
| 壁厚 | 0.038 m |
| PGM 解像度 | 0.05 m/cell |
| PGM 原点 X | -3.481 m（南西端外壁外側） |
| PGM 原点 Y | 0.0 m（南端外壁外側） |

**コート別スタート位置:**

| コート | x | y | yaw |
|--------|---|---|-----|
| blue（右） | 3.143 | 0.288 | -1.5708（南向き） |
| red（左） | -3.143 | 0.288 | 1.5708（北向き） |

### 3-4. waypoints.yaml フォーマット（on_arrive 対応版）

```yaml
auto_sequence:
  - waypoint_1
  - waypoint_2

waypoints:
  waypoint_1:
    x: 1.404       # フィールド座標 [m]、青コート基準
    y: 0.25
    theta: -1.5707  # ロボット向き [rad]（0=東、π/2=北、-π/2=南）
    on_arrive:
      - action: hand_control
        target: yagura_1
        control_type: pos
        action_value: up
      - action: wait
        duration: 1.5
  waypoint_2:
    x: 1.703
    y: 6.712
    theta: -1.5707
    on_arrive: []
```

**`on_arrive` アクション種別:**

| action | 必須フィールド | 説明 |
|--------|-------------|------|
| `hand_control` | `target, control_type, action_value` | bluetooth_rx に手動制御コマンドを発行 |
| `wait` | `duration` (秒) | 指定秒数待機（50ms チェック付き中断可能スリープ） |

**`target`:** `yagura_1`, `yagura_2`, `ring_1`, `ring_2`

| control_type | action_value の値 |
|-------------|---------------|
| `pos`（yagura） | `up`, `down`, `stopped` |
| `state`（yagura） | `open`, `close`, `stopped` |
| `pos`（ring） | `pickup`, `yagura`, `honmaru`, `stopped` |
| `state`（ring） | `open`, `close`, `stopped` |

---

## 4. 主要パラメータ

### 4-1. Nav2 パラメータ（nav2_params.yaml）

**コントローラー: MPPI（Omni モデル）**

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `controller_frequency` | 5.0 Hz | Raspberry Pi 負荷に合わせた設定 |
| `batch_size` | 300 | 軌跡サンプル数（負荷軽減のため削減） |
| `time_steps` | 15 | 予測ホライズン（0.2s×15=3.0s） |
| `model_dt` | 0.2 | コントローラー周期と一致させる |
| `visualize` | false | /trajectories 配信無効（負荷軽減） |
| `vx_max` | 2.5 | 前後最大速度 [m/s] |
| `vy_max` | 1.5 | 左右最大速度 [m/s]（メカナム横移動） |
| `wz_max` | 3.5 | 回転最大角速度 [rad/s] |

**ゴールチェッカー:**

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `xy_goal_tolerance` | 0.15 m | 位置精度目標 |
| `yaw_goal_tolerance` | 0.20 rad | 角度精度目標（~11°） |

**bt_navigator:**

| パラメータ | 値 | 理由 |
|-----------|-----|------|
| `default_server_timeout` | 1000 ms | CPU 処理落ち時の猶予（旧値 20ms で controller_server タイムアウト多発） |

**フットプリント:** `[[0.25, 0.29], [-0.25, 0.29], [-0.25, -0.29], [0.25, -0.29]]`（中心→前後 0.25m、中心→左右 0.29m）

**グローバルプランナー:** NavfnPlanner（A*: `use_astar: true`, tolerance: 0.5m）

**コストマップ:** `/scan_filtered` を使用、`inflation_radius: 0.40m`

### 4-2. AMCL パラメータ（amcl_params.yaml）

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `robot_model_type` | OmniMotionModel | 全方向移動ロボット用 |
| `min_particles` | 500 | 最小パーティクル数 |
| `max_particles` | 2000 | 最大パーティクル数 |
| `update_min_d` | 0.15 m | 更新トリガー（並進） |
| `update_min_a` | 0.10 rad | 更新トリガー（回転、~5.7°） |
| `z_rand` | 0.05 | ランダムノイズ割合（合成マップ向け低値） |
| `scan_topic` | scan_filtered | タイヤ映り込み除去済みスキャン |

**set_initial_pose: true** に設定してあるため、set_court コマンド前でも原点付近に初期化される（Nav2 が起動できる状態を保つため）。

### 4-3. 運動学定数（constants.py / odometry_node.py / cmd_vel_bridge_node.py 共通）

```
WHEEL_RADIUS = 0.04925 m
GEAR_RATIO   = 19.20320855614973
L_X = 0.1725 m（前後方向: ロボット中心 → タイヤ中心）
L_Y = 0.2425 m（左右方向: ロボット中心 → タイヤ中心）
G   = L_X + L_Y = 0.415 m
```

**逆運動学**（cmd_vel_bridge_node.py、ROS標準座標系: Vy>0=左）:

```
v_fl = +Vx - Vy - G*ω
v_fr = -Vx - Vy - G*ω
v_rl = +Vx + Vy - G*ω
v_rr = -Vx + Vy - G*ω
```

**前進運動学**（odometry_node.py）:

```
Vx    = ( v_fl - v_fr + v_rl - v_rr) / 4
Vy    = -( v_fl + v_fr - v_rl - v_rr) / 4
omega = -(v_fl + v_fr + v_rl + v_rr) / (4 * G)
```

FR/RR の wheel_feedback RPM は取付向きの符号反転がそのまま返るため、変換式に追加の符号処理は不要。

**MIN_RPM = 300.0**（M3508 静止摩擦対策。 ≈ 0.08 m/s）

---

## 5. Bluetooth / WebSocket コマンド仕様

### ロボットへの送信（bluetooth_rx）

| type | フィールド | 説明 |
|------|-----------|------|
| `nav_mode` | `mode: "auto"\|"manual"` | ナビゲーションモード切り替え |
| `nav_goal` | `waypoint: string` | ウェイポイント名でゴール指定 |
| `nav_goal` | `x, y, theta: float` | 直接座標でゴール指定 |
| `set_court` | `court: "blue"\|"red"` | コート選択（AMCL に /initialpose を送信） |
| `start_auto` | `from_index?: int` | auto_sequence を指定インデックスから実行（省略時=0） |
| `stop_auto` | — | 自動制御停止（シーケンス中断） |
| `joystick` | `l_x, l_y, r: int (-10〜10)` | 手動走行 |
| `hand_control` | `target, control_type, action` | ハンド操作 |
| `pid_gains` | `kp, ki, kd: float` | PID ゲイン調整 |

### ロボットからの受信（bluetooth_tx）

| 発生元 | フィールド | 説明 |
|--------|-----------|------|
| routing_node | `nav_status: "mode"`, `mode` | モード変更通知 |
| routing_node | `nav_status: "navigating"`, `waypoint`, `seq_index`, `seq_total` | 走行中 |
| routing_node | `nav_status: "arrived"`, `waypoint`, `seq_index`, `seq_total` | 到達通知（on_arrive 完了後） |
| routing_node | `nav_status: "completed"` | 全 auto_sequence 完了 |
| routing_node | `nav_status: "cancelled"` | キャンセル通知 |
| routing_node | `nav_status: "error"`, `message`, `seq_index`, `seq_total` | エラー通知 |
| routing_node | `nav_status: "relocating"`, `countdown`, `waypoint`, `seq_index`, `seq_total` | TF 確立待ち（最大30秒） |
| routing_node | `nav_status: "court_set"`, `court` | コート設定完了通知 |
| routing_node | `type: "robot_pos"`, `x`, `y`, `angle` | ロボット位置（5Hz、単位: mm・deg） |
| robot_control | `m3508_rpms`, `yagura`, `ring`, ... | フィードバック |

---

## 6. 既知の問題・注意事項

1. **LiDAR 取付位置の TF**: z=0.15m は仮設定。実機で計測して `auto_nav_launch.py` の static_tf を更新すること。

2. **AMCL 初期位置**: set_court コマンドを送ると routing_node が `/initialpose` を publish して AMCL に初期位置を伝える。競技前に必ず `set_court` を実行すること。start_auto 実行時も from_index に関わらず initialpose を送信し、map→odom TF が確立されるまで最大 30 秒待機する。

3. **メカナムホイールのスリップ**: スリップが多いとオドメトリ精度が悪化し AMCL 収束が遅くなる。可能な限りゆっくり走行すること。

4. **IMU gz バイアスキャリブレーション**: imu_publisher_node は起動時に静止状態で ~5 秒間 gz サンプルを収集してバイアスを推定する。起動直後は静止させること。キャリブレーション完了まで EKF は /odom_raw のみで動作する。

5. **IMU 加速度の有効化（未実施）**: LSM9DS1 の `ax`, `ay` は g 単位で m/s² 変換済みで配信しているが、EKF への入力は無効のまま（`imu0_config` で false）。有効化すると急加速時の精度が向上する可能性があるが、未検証。

6. **AMCL 更新頻度**: `update_min_d: 0.15`, `update_min_a: 0.10`（すでに改善済み）。さらに下げると CPU 負荷が増加する。

7. **on_arrive 実行中のブロッキング**: `time.sleep` を rclpy メインスレッドで呼ぶとコールバックが止まる。on_arrive シーケンスは専用スレッドで実行済み。

8. **手動/自動モード競合**: manual 時は robot_control が、auto 時は cmd_vel_bridge が wheel_control をパブリッシュ。`auto → manual` 切り替え時に cmd_vel_bridge がゼロ RPM を送信して急停止を防ぐ。

9. **asyncio + rclpy 統合**: web_server_node は aiohttp の asyncio ループと rclpy のループを別スレッドで動かす。`asyncio.run_coroutine_threadsafe` / スレッドセーフ publish 以外での跨ぎ操作は禁止。

10. **YDLiDAR のデッドゾーン**: 0.1m 以内はデッドゾーン。`LaserScanRangeFilter` で 0.33m 未満を除去（タイヤ映り込み対策も兼ねる）。`LaserScanFootprintFilter` は TF 変換でサイレント失敗するため不使用。

11. **laser_filter_node の name= 禁止**: `name=` を設定するとパラメータ読み込み失敗 → `name=` を削除してデフォルト名を使う。

12. **auto モードでの hand_control**: hand_control の送信は auto/manual 両モードで動作する（robot_control の wheel_control 送信のみ auto モード時にスキップ）。

13. **Nav2 ナビゲーション失敗時のリトライ**: auto sequence 中に Nav2 が失敗した場合、最大 5 回まで 3 秒間隔でリトライする。全リトライ後に error を送信する。

14. **colcon build --cmake-force-configure**: `file(GLOB)` でスクリプトを追加した際はキャッシュクリアが必要な場合がある。

### センサフュージョン改善メモ（今後の課題）

1. **IMU 加速度の有効化**（優先度: 中）: `ros2 topic echo /imu` で raw 値を確認し単位を実測したうえで `ekf_params.yaml` の `imu0_config` を有効化。急加速・急停止時のオドメトリ精度向上が期待できる。

2. **IMU orientation の差分利用**（優先度: 低）: 磁気干渉で orientation が不正確なため現在は無効。`imu0_differential: true` に変更すると干渉の影響を受けにくくなる可能性がある。

---

## 7. 必要パッケージ

```bash
sudo apt install \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-nav2-map-server \
  ros-jazzy-laser-filters \
  ros-jazzy-robot-localization
```

### web_control セットアップ（初回のみ）

```bash
cd /home/pi/DRC/khr2026_team1_rspi/src/web_control
uv venv --system-site-packages
uv add aiohttp
cd /home/pi/DRC/khr2026_team1_rspi
colcon build --packages-select web_control --symlink-install
source install/setup.bash
```

---

## 8. よく使うデバッグコマンド

```bash
# AMCL 自己位置確認
ros2 topic echo /amcl_pose

# map→odom TF 確認（AMCL が正常動作していれば map フレームが現れる）
ros2 run tf2_tools view_frames

# /map トピック確認
ros2 topic echo --qos-reliability reliable /map nav_msgs/msg/OccupancyGrid --field info

# lifecycle_manager_localization 状態確認
ros2 service call /lifecycle_manager_localization/is_active std_srvs/srv/Trigger

# nav_mode 切り替え
ros2 topic pub /nav_mode std_msgs/String "data: 'auto'" --once
ros2 topic pub /nav_mode std_msgs/String "data: 'manual'" --once

# routing_node テスト
ros2 topic echo /bluetooth_tx
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"' --once
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"set_court\", \"court\": \"blue\"}"' --once
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"start_auto\"}"' --once

# WS ブリッジ確認
ros2 topic echo /bluetooth_rx
ros2 topic pub /bluetooth_tx std_msgs/String '{data: "{\"nav_status\": \"test\"}"}' --once

# cmd_vel テスト（auto モード時）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once

# MPPI 動作確認（5Hz 付近で安定出力されていれば OK）
ros2 topic hz /cmd_vel

# yagura_position_node 確認（距離近い順に _0/_1/_2）
ros2 topic echo /yagura_position_0
ros2 topic echo /yagura_position_1
ros2 topic echo /yagura_position_2
```

---

## 9. 競技作戦

### 作戦概要

機体を 700×700mm スタートゾーンに収めた状態から開始。

| ステップ | 内容 | ウェイポイント | 使用ハンド |
|---------|------|-------------|-----------|
| ① | 櫓2つ取得 | `yagura_pickup_1` | yagura_1, yagura_2 |
| ② | リング4本取得（各ハンド2本） | `ring_pickup_1` | ring_1, ring_2 |
| ③-1 | 櫓1 + リング2本を配置 | `yagura_release_1` | yagura_1, ring_1 |
| ③-2 | 櫓2 + リング2本を配置 | `yagura_release_2` | yagura_2, ring_2 |
| ④-a | リング3本取得（ring_1: 2本, ring_2: 1本） | `ring_pickup_2` | ring_1, ring_2 |
| ④-b | 櫓1つ取得 | `yagura_pickup_2` | yagura_1 |
| ⑤ | 櫓 + リング2本を配置（王手達成） | `yagura_release_3` | yagura_1, ring_1 |
| ⑥ | 本丸にリングを通す（攻略達成） | `honmaru` | ring_2 |

### 櫓+リング配置手順（③・⑤共通）

1. リングを櫓位置へ移動（`ring pos=yagura`）
2. 櫓を下ろす（`yagura pos=down`）
3. 櫓を離す（`yagura state=open`）
4. リングをリリース（`ring state=open`）
5. 櫓機構を退避（`yagura pos=up`）

### フォールバック

- 全自動機構が動作しなかった場合、1回のみリトライ後、半自動（手動）制御に切り替え
- 相手側からの妨害投げ込みはセンサで検知し、邪魔にならない位置まで排除

---

## 10. 作業記録（最新が上）

### 2026-03-10

- **`yagura_approach` アクション追加**（`routing_node.py`）
  - **概要**: `on_arrive` シーケンス内で `yagura_position_0` トピックの検出座標を使い、LiDAR 検出した櫓に対して自動アプローチするネスト航行アクション
  - **仕組み**:
    1. `yagura_position_0`（`geometry_msgs/PointStamped`）から最近傍櫓の base_link 座標を取得
    2. TF で map 座標に変換し、ハンドオフセット（`hand_offset_x=0.35m`, `hand_offset_y=0.14m`）を加算してゴール座標を算出
    3. `_approach_done_event`（`threading.Event`）を使い NavigateToPose を発行してシーケンススレッドで完了を待機
    4. `theta = -π/2`（南向き）固定
  - **新フィールド**: `_yagura_pos`, `_yagura_pos_lock`, `_sub_yagura`, `_approach_done_event`, `_approach_success`
  - **新コールバック**: `_on_yagura_pos()`
  - **`_result_cb` 変更**: `_approach_done_event` が設定されている場合はシーケンスへの通知のみ行い早期リターン
  - waypoints.yaml の `on_arrive` に `action: yagura_approach` で使用可

  ```yaml
  on_arrive:
    - action: yagura_approach
      approach_dist: 0.35    # 未使用（将来用）
      hand_offset_x: 0.35    # 西方向オフセット [m]
      hand_offset_y: 0.14    # 北方向オフセット [m]
      timeout: 30.0
  ```

- **DIRECT_APPROACH 強化: theta P 制御 + LiDAR 補完停止**（`routing_node.py`）
  - **背景**: 従来の DIRECT_APPROACH は単純な xy 直線移動のみ。ウェイポイント到達後の向き（theta）を無視し、かつ AMCL 誤差があると停止精度が低かった
  - **変更内容**:
    1. `MIN_RPM_FWD` 1500→**600**、`MIN_RPM_LAT` 3000→**800**
    2. DIRECT_APPROACH 中の MIN_RPM 底上げを削除（P 制御のみで速度を決定し自然に減速）
    3. 並進 P 制御（`KP_POS=1.0`）+ 角速度 P 制御（`KP_YAW=2.0`）を同時実行
    4. ゴール到達判定に `yaw_error ≤ 0.08 rad` 条件を追加
    5. `/scan_filtered` を購読し、進行方向前方 ±20° の最小距離 ≤ 0.05m で補完停止
    6. 逆運動学の `vy` に 1.35x スリップ補償を適用（cmd_vel_bridge_node と同様）
  - **追加定数**: `KP_POS`, `KP_YAW`, `MAX_OMEGA`, `YAW_TOLERANCE`, `LIDAR_FORWARD_ANGLE`, `LIDAR_STOP_DIST`
  - **新メソッド**: `_on_scan()`, `_lidar_forward_min_dist()`
  - **速度過大の原因と対策**: MIN_RPM 底上げ（旧 1500/3000 RPM）が 0.15 m/s 指令を 0.4〜0.8 m/s に拡大していたため、DIRECT_APPROACH 中は底上げを無効化した
  - ビルド不要（symlink-install 済み、ノード再起動のみ）

  確認ログ:
  ```
  "Direct approach activated (dist=X.XXXm)"
  "Goal reached (direct approach, dist=X.XXXm, yaw_err=X.XXX)"  ← AMCL 停止
  "Goal reached by LiDAR (forward_dist=X.XXXm)"                 ← LiDAR 停止
  ```

  調整パラメータ（実機テストで要確認）:
  - `KP_YAW = 2.0`: 振動するなら小さく、角度残留が大きいなら大きく
  - `YAW_TOLERANCE = 0.08 rad`: 厳しすぎると到達しない（壁際では緩めに）
  - `LIDAR_STOP_DIST = 0.05 m`: 開放域では前方に障害物がないので無効、壁際のみ有効

### 2026-03-09（2回目）

- **メカナム横滑り補正（スリップ係数 1.35）の導入**
  - **原因**: 「特定の角度だけ成功し、横移動・斜め移動で混乱した動き」という症状は、メカナムホイールの横方向スリップによる Odometry と AMCL の喧嘩が原因
    - 横移動時にローラーが空転し、実移動 ≈ エンコーダ換算値 × 0.741
    - オドメトリは「完璧に進んだ」と誤報告 → AMCL がワープ補正 → Nav2 がパニック → 混乱した動き
    - 前後移動はタイヤがグリップするため成功、特定角度問題の原因はこれ
  - **修正箇所**:
    - `cmd_vel_bridge_node.py`: `vy = msg.linear.y * 1.35`（スリップ分を増幅して実移動を補償）
    - `odometry_node.py`: `vy = ... * 0.741`（エンコーダ換算値を実移動量に合わせて縮小）
  - **2変更の意味**:
    - 指令側(1.35倍): 実移動 = 指令値 × slip_ratio(0.741) × 1.35 ≈ 指令値通り
    - オドメトリ側(0.741倍): 報告値 ≈ 実移動量 → AMCL と一致
  - **調整可能範囲**: 床の摩擦・ロボット重量により 1.2〜1.5 の間で微調整
  - ビルド不要（symlink-install 済み、ノード再起動のみ）

  確認コマンド:
  ```bash
  # 横移動テスト（vy 指令が底上げされず小さい値で送られることを確認）
  ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {z: 0.0}}" --rate 5
  ros2 topic echo /wheel_control

  # 前進テスト（FL/RL 正値、FR/RR 負値で変化なし）
  ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --rate 5
  ```

- **`cmd_vel_bridge_node.py` から MIN_RPM 底上げ・HeadingPID を削除**
  - **原因（MIN_RPM）**: Nav2 微小指令（例: 0.01 m/s）を強制 300 RPM に拡大 → Nav2 クローズドループ破綻
  - **原因（HeadingPID）**: Nav2 が自分で閉ループ制御しているのに下位ノードが独自ヨー補正 → 制御の二重掛け
  - **修正**: 両方のブロックを削除し、不要インポート・フィールドも全削除
  - ビルド不要

### 2026-03-09

- **MPPI 負荷軽減 & bt_navigator タイムアウト緩和**（`nav2_params.yaml`）
  - **原因**: MPPI 計算量が Raspberry Pi の処理限界を超え、`bt_navigator` が 20ms タイムアウト内に `controller_server` から応答を受け取れず `follow_path` アクションが中断される不具合
  - **対応**:
    - `default_server_timeout`: 20 → 1000 ms（CPU 処理落ち時の猶予）
    - `batch_size`: 800 → 300（Raspberry Pi の処理能力に合わせた上限）
    - `time_steps`: 30 → 15（ホライズン 6.0s → 3.0s）
    - `visualize`: true → false（/trajectories 配信を無効化、負荷軽減）
    - `model_dt` は 0.2 のまま維持（controller_frequency=5Hz 相当）
  - **計算量**: `800×30 = 24,000` → `300×15 = 4,500` 回/ループ（約 1/5）
  - ビルド不要（yaml のみ変更）

  確認コマンド:
  ```bash
  ros2 launch auto_nav auto_nav_launch.py
  ros2 topic pub /nav_mode std_msgs/String "data: 'auto'" --once
  ros2 topic hz /cmd_vel  # 5Hz 付近で安定出力されていれば OK
  ```

### 2026-03-08（3回目）

- **`cmd_vel_bridge_node.py` の逆運動学 `vy` 符号バグ修正**
  - **原因**: Nav2 が `/cmd_vel` に出力する `vy > 0`（ROS標準=左移動）に対して、逆運動学式の `vy` 符号が逆だった
    - `vx=0, vy=+1` を入力すると `v_fl=+1, v_fr=+1, v_rl=-1, v_rr=-1` → odom の前進運動学で Vy_actual = -1（右移動）
    - Nav2 が経路補正のため「左へ」と指令するほどロボットは右へ逸れ、誤差が拡大して壁に激突していた
  - **なぜ手動モードで気づかなかったか**: `m3508.py` が内部で `vy = -rotated_velocity.y` と反転しているため、手動モードではジョイスティック入力が偶然に正しく動作していた（符号反転の二重打ち消し）
  - **修正箇所**: `cmd_vel_bridge_node.py` の逆運動学式
    - 修正前: `v_fl = +vx + vy`, `v_fr = -vx + vy`, `v_rl = +vx - vy`, `v_rr = -vx - vy`
    - 修正後: `v_fl = +vx - vy`, `v_fr = -vx - vy`, `v_rl = +vx + vy`, `v_rr = -vx + vy`
  - **検算**: `vy=+1` で `v_fl=-1, v_fr=-1, v_rl=+1, v_rr=+1` → Vy_actual = -(-1-1-1-1)/4 = +1 ✓

### 2026-03-08（2回目）

- **AMCL 移行後の各種修正・整理**
  - `field_dimensions.yaml`: マップ固定（外寸方式）に伴い全寸法を実測値に更新
    - `width: 6.962`, `height: 7.0`, `wall_thickness: 0.038`
    - `origin.y: 0.0`（フィールド南端外壁外側が y=0）
    - `start_positions.y: 0.288`（南壁内側面 0.038 + ロボット半径 0.25）
    - `internal_walls` に仕切り壁・矢倉・エリア壁・本丸を追加
  - `generate_field_map.py`: `internal_walls` 描画ロジックを実装
  - `mapping_launch.py` 削除（AMCL 移行により不要）
  - `generate_waypoints.py` 削除・`waypoints_relative.yaml` 削除（マップ固定により相対座標変換が不要に）
  - `waypoints.yaml` を直接編集ファイルに変更（絶対座標で記述）
  - `generate_field_map.py` の docstring・print から `generate_waypoints.py` 参照を削除

- **AMCL デバッグ**
  - `map_server` の `yaml_filename` パラメータが `.yaml` 拡張子なしで渡される問題を確認 → `auto_nav_launch.py` 起動時は `map:=/home/pi/maps/field_synthetic.yaml` を明示すること（デフォルト値に拡張子付きで設定済み）
  - AMCL は `set_court` コマンド受信まで `/initialpose` を得られず `map → odom` TF を publish しない（正常動作）

- **センサフュージョン改善メモを auto_routing_plan.md に追記**

### 2026-03-08（1回目）

- **フェーズ 12**: 事前フィールドマップ + AMCL 自己位置推定への移行
  - slam_toolbox (localization) と scan_relay_node を廃止
  - `generate_field_map.py` でルールブック寸法から合成 PGM/YAML を生成するスクリプトを追加
  - `amcl_params.yaml` を新規作成（OmniMotionModel、scan_topic=scan_filtered）
  - `nav2_params.yaml` に `lifecycle_manager_localization`（map_server + amcl）セクションを追加
  - `auto_nav_launch.py`: slam_toolbox/scan_relay → map_server + amcl + lifecycle_manager_localization（8秒遅延起動）
  - `routing_node.py`: `set_court` 受信時に `field_dimensions.yaml` の `start_positions` から `/initialpose` を publish
  - `colcon build --cmake-force-configure` が必要だった（新スクリプトを file(GLOB) で拾うためキャッシュクリアが必要）
  - 合成マップ検証: `generate_field_map.py` → 148×80px PGM 生成確認

### 2026-03-07

- **点群回転問題の修正（RViz 上で壁に点群が固定されない問題）**
  - **LiDAR TF 修正** (`auto_nav_launch.py`):
    - 旧: `--yaw 1.5708 --pitch 3.14159 --roll 0.0`
    - 新: `--yaw 1.5708 --pitch 0.0 --roll 0.0`
    - pitch=π（Y軸180°回転）は X軸を反転させるため、2D スキャン上で左右鏡像を引き起こす。試行錯誤の結果 `yaw=π/2, pitch=0, roll=0` が前後・左右ともに正しいことを実機で確認。
  - **odometry_node.py の vy 符号修正**:
    - 旧: `vy = (v_fl + v_fr - v_rl - v_rr) / 4.0`
    - 新: `vy = -(v_fl + v_fr - v_rl - v_rr) / 4.0`
    - 右移動時に /odom_raw の vy が正（左方向）になっていたため符号反転。
  - **確認手順**:
    1. `ros2 topic echo /imu --field angular_velocity.z` + 左回転 → 正の値
    2. `ros2 topic echo /odom_raw --field twist.twist.angular.z` + 左回転 → 正の値
    3. `ros2 topic echo /odom --field twist.twist.angular.z` + 左回転 → 正の値
    4. RViz (Fixed Frame=odom) でロボット旋回 → 点群が壁に固定される

### 2026-03-05

- **直進蛇行・角度ずれ修正**: HeadingPID の kp が度単位 error に rad/s スケールで掛かる問題を修正
  - 根本原因: `heading_pid.py` は error を度単位で計算するが output を rad/s として IK に渡すため kp=1.0 は実効的に 57 倍の大きさで動作し、1° ドリフト → ~1548 RPM の過剰補正
  - **Fix 1** (`cmd_vel_bridge_node.py`): `kp=1.0, kd=0.05` → `kp=0.02, kd=0.001`
  - **Fix 2** (`cmd_vel_bridge_node.py`): `OMEGA_THRESHOLD=0.05` → `0.02` rad/s
  - **Fix 3** (`imu_publisher_node.py`): gz 符号診断ログを追加（旋回中に `ratio=gz/omega_enc` を DEBUG 出力、ratio≈+1.0 → 正常、ratio≈-1.0 → gz 反転が必要）

### 2026-03-04

- **フェーズ 11**: エラーアラートとシーケンス途中再開を実装
  - `routing_node.py`: `start_auto` に `from_index` オプション追加、`_relocate_and_start()` スレッドで initialpose publish + map→odom TF 確立まで最大 30 秒待機（1 秒ごとに `nav_status: "relocating"` を送信）後にナビ開始、navigating/arrived/error メッセージに `seq_index`/`seq_total` 追加
  - `useAutoNav.ts`: `isAlertFlashing`/`failedSeqIndex`/`relocatingCountdown` 状態追加、`sendStartAutoFrom(fromIndex)` 追加、`RELOCATING` ステータス追加
  - `AutoNav/index.tsx`: 赤白交互点滅オーバーレイ（error 時）、シーケンスリストに「ここから」ボタン、リローカライズカウントダウン表示、from_index > 0 の確認ダイアログ追加

  **競技時のリトライフロー:**
  ```
  ① エラー発生 → 画面が赤白点滅（pointer-events-none なのでボタン操作は継続可）
  ② オペレーター: 「停止」ボタン → 手動でロボットを対象ウェイポイント位置に移動
  ③ シーケンスリストで失敗行（オレンジ）の「ここから」ボタンを押す
  ④ 確認ダイアログ → OK
  ⑤ rspi が /initialpose を publish → TF 確立待ち（最大 30 秒、残秒数を UI に表示）
  ⑥ ナビゲーション再開（点滅も停止）
  ```

### 2026-03-03

- **手動制御コート選択機能追加**: `Controller.tsx` にコート選択ボタン（青/赤）を追加
  - 青コート選択時: l_x・l_y を符号反転（ロボットが南向きのため）
  - 赤コート選択時: 変換なし（ロボットが北向きのため）
  - コート選択時に `set_court` コマンドを送信

- **バグ修正**: `routing_node.py` の `_apply_court_transform` の赤コート変換が誤り
  - 誤: `(x, -y, -θ)` ← 東西線（横線）対称になっていた
  - 正: `(-x, y, π-θ)` ← 南北線（縦線）対称（フィールドは赤=左・青=右コートが横並び）

### 2026-03-02

- **フェーズ 9**: routing_node.py に on_arrive シーケンス・start_auto/stop_auto・set_court・コート座標変換を追加
- ros2_node.py: hand_control は auto/manual 両モードで送信するよう変更（wheel_control のみ auto モード時にスキップ）

- **フェーズ 8**: WebSocket コントロールシステム実装
  - `src/web_control/` パッケージ新規作成（aiohttp WebSocket ブリッジ、port 8080）
  - `auto_nav_launch.py` に `web_server_node` を追加（`_WEB_PYTHONPATH` で aiohttp venv を参照）
  - bt_controller: `useWebSocketConnect.tsx`（WS 接続管理フック）
  - bt_controller: `CourtSelector.tsx`（コート選択コンポーネント）
  - bt_controller: `useAutoNav.ts` をリファクタリング（引数を `sendJson` に抽象化）
  - bt_controller: `AutoNav/index.tsx` を更新（BLE/WS タブ切り替え、コート選択、自動シーケンス制御 UI）

### 2026-03-01

- **フェーズ 10**: `generate_waypoints.py` + `waypoints_relative.yaml` 実装（PGM 純粋 Python 読み込み、P2/P5 両対応、`--dry-run` オプション付き）→ **2026-03-08 にマップ固定化に伴い削除**

### 2026-02-27

- **EKF センサフュージョン導入**: odometry_node.py から IMU 融合・TF 配信を除去し `/odom_raw` のみに変更。imu_publisher_node.py（新規）+ ekf_params.yaml（新規）を追加
- SLAM マッピングパラメータ調整（対角壁重複描画・二重壁解消）
- **routing_node.py 実装**: ActionClient + SingleThreadedExecutor で bluetooth_rx コールバックが無音になる問題 → MultiThreadedExecutor に変更で解決
- DWB チューニング + MIN_RPM 追加 → **2026-03-09 時点では MPPI に移行済み**

### 2026-02-25

- `nav2_params.yaml` 新規作成（Nav2 組み込み）
- Nav2 Jazzy 固有のトラブル解決: plugin_lib_names の二重登録、docking_server/collision_monitor のパラメータ追加

### 2026-02-23

- コードレビュー修正（slam params の enable_interactive_mode/do_loop_closing、odometry_node の theta 積分順序修正・共分散行列設定）
- 実機マッピング成功

### 2026-02-22

- /odom 配信 20Hz、モード切り替え正常確認
- scripts/*.py に実行権限がなかった → `chmod +x` で修正（スクリプト追加時は必ず実行権限を付与すること）
- `LaserScanRangeFilter` で /scan_filtered 実装（LaserScanFootprintFilter は TF 変換でサイレント失敗するため不使用）
- slam_toolbox ライフサイクル問題解決: `ros2 service call /slam_toolbox/change_state` を直接呼び出し（デーモン経由不可）

### 2026-03-09

- **CPU 軽量化（フェーズ 1・2 全適用済み）**

  | ファイル | 変更箇所 | 旧値 → 新値 |
  |---------|---------|------------|
  | `field_dimensions.yaml` | resolution | 0.05 → 0.10 |
  | `nav2_params.yaml` | global_costmap.resolution | 0.05 → 0.10 |
  | `nav2_params.yaml` | local_costmap.resolution | 0.05 → 0.10 |
  | `nav2_params.yaml` | local_costmap.width/height | 3 → 2 |
  | `nav2_params.yaml` | local_costmap.update_frequency | 5.0 → 3.0 Hz |
  | `nav2_params.yaml` | local_costmap.publish_frequency | 2.0 → 1.0 Hz |
  | `nav2_params.yaml` | MPPI batch_size | 300 → 150 |
  | `nav2_params.yaml` | MPPI time_steps | 15 → 12 |
  | `nav2_params.yaml` | MPPI iteration_count | 2 → 1 |
  | `amcl_params.yaml` | max_particles | 2000 → 800 |
  | `amcl_params.yaml` | update_min_d | 0.15 → 0.20 m |
  | `amcl_params.yaml` | update_min_a | 0.10 → 0.20 rad |

  - `generate_field_map.py` を再実行して PGM 再生成済み（140×140px → 70×70px）
  - **次回: 実機で CPU 使用率・ゴール到達精度（< 5cm）を確認すること**

  ```bash
  # 検証コマンド
  top -d 1                          # CPU 使用率確認（別ターミナル）
  ros2 launch auto_nav auto_nav_launch.py
  ros2 topic hz /cmd_vel            # 制御周期の乱れ確認
  ros2 topic echo /global_costmap/costmap_updates | head -5  # 解像度確認
  ```

  - 軽量化後も精度不足なら MPPI パラメータを元に戻す（batch_size: 300, time_steps: 15, iteration_count: 2 が実績値）

- **壁際ウェイポイント直接アプローチ機能追加（`routing_node.py`）**

  Nav2 のコストマップインフレーションにより壁際ゴールで MPPI が有効軌道を見つけられない問題への対策。

  **仕組み**: `NAVIGATING` 中に 50ms タイマーで距離監視 → 0.5m 以下で Nav2 キャンセル → `DIRECT_APPROACH` 状態に遷移 → TF lookup で位置取得 → P制御で `wheel_control` に直接 publish → 0.05m 以下で到達判定

  | 定数 | 値 | 説明 |
  |------|-----|------|
  | `DIRECT_APPROACH_DIST` | 0.5 m | Nav2 → 直接アプローチ切替距離 |
  | `GOAL_REACHED_DIST` | 0.05 m | ゴール到達判定距離 |
  | `DIRECT_APPROACH_SPEED` | 0.15 m/s | 直接アプローチ移動速度 |

  **変更詳細**:
  - `WheelMessage` import + 運動学定数追加（`cmd_vel_bridge_node.py` と同じ値）
  - `wheel_control` publisher + `_approach_goal_xy` + 50ms タイマー `_on_approach_timer` 追加
  - `_send_goal` で `_approach_goal_xy` にゴール座標をセット
  - `_result_cb`: `STATUS_CANCELED` 時に `DIRECT_APPROACH` 状態なら何もしない（タイマー側で制御）
  - `_handle_mode`/`_handle_stop_auto`: `DIRECT_APPROACH` 時のクリーンアップ（ゼロ RPM 送信 + 状態リセット）
  - マーカー表示: `DIRECT_APPROACH` も current 判定に追加
  - MIN_RPM 底上げなし（壁際で暴走防止）

  ```bash
  # 検証コマンド
  ros2 launch auto_nav mapping_launch.py   # or auto_nav_launch.py
  # 壁際ウェイポイントへの自動走行を実行し、以下のログを確認:
  #   "Direct approach activated (dist=X.XXXm)"
  #   "Goal reached (direct approach, dist=X.XXXm)"
  ```

  - **注意**: `DIRECT_APPROACH_DIST`（0.5m）が小さすぎると MPPI が先に詰まる。大きすぎると経路計画の恩恵を失う。実機テストで要調整。

- **直接アプローチ MIN_RPM 修正 + MPPI time_steps 動的変更 + 計算量増加**

  直接アプローチで RPM がモーターデッドゾーン以下（~559 RPM）になりロボットが傾いて停止する問題を修正。
  併せて MPPI パラメータを距離に応じて動的変更する仕組みを追加。

  **MIN_RPM 修正（`routing_node.py`）**:
  - 専用定数ではなく `cmd_vel_bridge_node.py` と共通の `MIN_RPM_FWD=1500` / `MIN_RPM_LAT=3000` + lateral_ratio ロジックを使用
  - 実効速度: 前進時 ~0.40 m/s、横移動時 ~0.81 m/s（0.5m 以内なので許容範囲）

  **MPPI time_steps 動的変更（`routing_node.py` + `nav2_params.yaml`）**:
  - `/controller_server/set_parameters` サービスで `FollowPath.time_steps` をランタイム変更
  - `_current_time_steps` で現在値を管理し、変更時のみサービスコール

  | 距離 | time_steps | ホライズン |
  |------|-----------|-----------|
  | > 1.5m | 8 | 1.6s（2.5 m/s で 4.0m 先まで見通し） |
  | ≤ 1.5m | 3 | 0.6s（壁際コスト影響を軽減） |
  | ≤ 0.5m | 直接アプローチ | Nav2 不使用 |

  **MPPI 計算量増加（`nav2_params.yaml`）**:
  - `iteration_count`: 2 → 3（最適化反復回数増加で軌道の質向上）
  - `time_steps`: 5 → 8（遠方時の初期値）
  - 計算量: 5×2=10 → 8×3=24（遠方）/ 3×3=9（近傍）

  ```bash
  # 検証コマンド
  ros2 launch auto_nav auto_nav_launch.py
  top -d 1  # CPU 使用率確認（別ターミナル）
  # ログで以下を確認:
  #   "MPPI time_steps → 3"  (ゴール 1.5m 以内)
  #   "MPPI time_steps → 8"  (次ゴール送信時 / 直接アプローチ移行時)
  #   "Direct approach activated" → "Goal reached (direct approach)"
  ```

### 2026-02-21

- `auto_nav` パッケージ新規作成
- `odometry_node.py`: wheel_feedback → /odom + TF(odom→base_link)
- `cmd_vel_bridge_node.py`: /cmd_vel → wheel_control、/nav_mode でモード切り替え
- `mapping_launch.py` 作成 → **2026-03-08 に AMCL 移行に伴い削除**
