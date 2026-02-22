# ROS2 + LiDAR 自律走行 実装計画

## 概要

既存のロボット制御システム（4輪オムニドライブ + YDLiDAR）に自律走行機能を追加する。
競技フィールドは毎回 ±5% の寸法誤差があるため、**試合前に毎回マッピング**を行い、
その試合の実際のフィールド形状に合わせた地図を生成してから自律走行する。

---

## 現在のシステム構成

```
[Bluetooth Controller]
        ↓ bluetooth_rx (JSON)
[RobotController Node]  ←── wheel_feedback / hand_feedback
        ↓
wheel_control / hand_control ──→ [ESP32] ──→ M3508モーター × 4

[YDLiDAR Driver] ──→ /scan (LaserScan)
```

**既存トピック:**
- `/scan` - LiDARスキャンデータ (sensor_msgs/LaserScan, 10Hz)
- `wheel_control` / `wheel_feedback` - 車輪速度制御 (WheelMessage)
- `bluetooth_rx` / `bluetooth_tx` - Bluetooth通信

---

## 目標アーキテクチャ

```
[目標位置指定]
  └── Bluetooth (JSON: {"type": "nav_goal", "waypoint": "goal_A"})
              ↓
[AutoRoutingNode]  ←── /map, slam_toolboxの自己位置
  ├── Nav2 ActionClient (NavigateToPose)
  └── /cmd_vel ──→ [CmdVelBridgeNode] ──→ wheel_control ──→ [ESP32]

[SLAM Toolbox (online_async)] ←── /scan, /odom ──→ /map, /tf(map→odom)
[OdometryNode]                ←── wheel_feedback ──→ /odom, /tf(odom→base_link)
[YDLiDAR]                     ──→ /scan
```

---

## 競技運用フロー（毎試合の流れ）

```
① 試合前マッピング（手動走行）
   ロボットを Bluetooth で操縦しながらフィールドを一周
   slam_toolbox が /scan + /odom から地図を自動生成
        ↓ 約1〜2分
② 地図保存
   マッピング完了後、地図をファイルに保存
   ウェイポイントもこの地図座標系で定義済みのものを使用
        ↓
③ ローカリゼーションモードへ切り替え
   slam_toolbox を localization モードで再起動
   保存した地図を読み込んで自己位置推定を開始
        ↓
④ 試合開始 → 自律走行
   Bluetooth で "nav_mode: auto" + "waypoint: goal_A" を送信
   Nav2 が経路計画 → /cmd_vel → 車輪制御
```

**フィールド誤差への対応:**
- 事前に設計図上の寸法でウェイポイントを定義するのではなく、
  **マッピング後に実際の地図上でウェイポイント座標を確認・調整**する
- 試合直前のマッピングで実フィールドの壁・障害物を取り込むため、
  ±5% の寸法誤差は地図に自動的に反映される

---

## フェーズ別実装計画

### フェーズ 1: オドメトリノード（優先度: 最高）

Nav2 と SLAM には `/odom` と TF（`odom → base_link`）が必須。
車輪フィードバックから推定位置・姿勢を計算するノードを実装する。

**新規パッケージ:** `src/auto_nav/`

**ファイル:** `src/auto_nav/src/auto_nav/odometry_node.py`

```python
# サブスクライブ: wheel_feedback (WheelMessage)
# パブリッシュ: /odom (nav_msgs/Odometry), /tf (odom→base_link)

# 4輪オムニの前進運動学:
#   Vx = (v_fl + v_fr + v_rl + v_rr) / 4
#   Vy = (-v_fl + v_fr + v_rl - v_rr) / 4
#   ω  = (-v_fl + v_fr - v_rl + v_rr) / (4 * (Lx + Ly))
#
# 車輪速度換算:
#   wheel_rpm → m/s: v = rpm / 60 * 2π * R / gear_ratio
#   R = 0.04925m, gear_ratio = 19.203
#
# 積分によるx, y, θの更新（デッドレコニング）
#   x   += (Vx*cosθ - Vy*sinθ) * dt
#   y   += (Vx*sinθ + Vy*cosθ) * dt
#   θ   += ω * dt
```

**パラメータ:**
- `publish_rate`: 50 Hz（wheel_feedbackと同期）
- `base_frame_id`: `base_link`
- `odom_frame_id`: `odom`

#### 実機動作確認（フェーズ1）

**目的:** オドメトリの計算が正しく、TF が正常に配信されているか確認する。

```bash
# 1. ノード起動（既存の robot_control + bt_communication も合わせて起動）
ros2 launch auto_nav mapping_launch.py

# 2. TFツリーが正しく構成されているか確認
ros2 run tf2_tools view_frames
# 期待: map は無くてよい。odom → base_link が存在すること

# 3. /odom が配信されているか確認
ros2 topic hz /odom        # 約50Hzで配信されているか
ros2 topic echo /odom      # x, y, theta が表示されるか
```

**確認テスト:**

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| 直進テスト | ロボットを前方に1m走らせる | `/odom` の `x` が約1.0mになる |
| 回転テスト | その場で360°回転させる | `theta` が約0に戻る（誤差 ±0.1rad以内） |
| 正方形テスト | 1m四方の正方形を走らせる | 終点がスタートから約20cm以内に収まる |

**よくある問題:**
- `/odom` が配信されない → `wheel_feedback` が来ているか `ros2 topic echo wheel_feedback` で確認
- 方向が逆 → FL/FR/RL/RR の符号設定を確認
- 位置が実際より大きい/小さい → `gear_ratio` または `wheel_radius` の値を確認

#### 実施結果（2026-02-21 完了）

**ステータス: ✅ 完了**

| 確認項目 | 結果 |
|----------|------|
| `/scan` 配信 | ✅ 正常 |
| `/odom` 配信 | ✅ 正常 |
| TF `odom → base_link` | ✅ 正常（`view_frames` で確認済み） |
| TF 配信レート | 20.2 Hz（wheel_feedback が ESP32 から ~20Hz で送られてくるため。計画の50Hzとは異なるが動作上は問題なし） |

**備考:**
- `odometry_node.py` の `publish_rate` パラメータは実際には wheel_feedback の受信レートに依存する（タイマーではなくサブスクライバーで駆動）
- SLAM（slam_toolbox）は 10Hz 程度のオドメトリでも動作するため、20Hz で十分

---

### フェーズ 2: cmd_vel → wheel_control 変換ノード（優先度: 高）

Nav2 は `/cmd_vel` (geometry_msgs/Twist) でロボットを制御する。
これを既存の `wheel_control` (WheelMessage) に変換するノードを実装する。

**ファイル:** `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py`

```python
# サブスクライブ:
#   /cmd_vel (geometry_msgs/Twist)
#   /nav_mode (String) - "manual" or "auto"
# パブリッシュ:
#   wheel_control (WheelMessage)

# 逆運動学（既存 m3508.py と同様）:
#   v_fl = Vx - Vy - (Lx + Ly) * ω
#   v_fr = Vx + Vy + (Lx + Ly) * ω
#   v_rl = Vx + Vy - (Lx + Ly) * ω
#   v_rr = Vx - Vy + (Lx + Ly) * ω
#   → RPM換算: rpm = v * 60 / (2π * R) * gear_ratio

# モード管理:
#   "auto" のとき: /cmd_vel を wheel_control に変換して出力
#   "manual" のとき: このノードは wheel_control を出力しない
#   (manual時は既存 RobotController が bluetooth_rx → wheel_control を担当)
```

**注意:** `manual` モードと `auto` モードで `wheel_control` の書き込み元が変わるため、
どちらか片方のみがパブリッシュする設計にすること。競合すると誤動作する。

#### 実機動作確認（フェーズ2）

**目的:** `/cmd_vel` を受け取ったときに実際に車輪が正しく動くか、モード切り替えが安全に動作するか確認する。

```bash
# 1. auto モードに切り替えてから手動で cmd_vel を送信してテスト
ros2 topic pub /nav_mode std_msgs/String "data: 'auto'" --once

# 前進指令: Vx=0.2m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 横移動指令: Vy=0.2m/s（オムニ確認）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 回転指令: ω=0.5rad/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

**確認テスト:**

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| 前進 | Vx=0.2 を送信 | ロボットが前方に動く。4輪が同じ方向に回転 |
| 横移動 | Vy=0.2 を送信 | ロボットが真横に動く（オムニ動作確認） |
| 回転 | ω=0.5 を送信 | ロボットがその場で回転する |
| モード競合テスト | `manual` モード中に cmd_vel を送信 | 車輪が動かないこと（Bluetooth操縦のみ有効） |
| モード切り替え | `manual` → `auto` → `manual` を繰り返す | 切り替え時に車輪が急動作しないこと |

**よくある問題:**
- 横方向に動かない → 逆運動学の Vy 項の符号を確認
- モード切り替え時に車輪が急動作する → 切り替え時に `/cmd_vel` をゼロ送信するロジックを追加

#### 実施結果（2026-02-21 完了）

**ステータス: ✅ 完了**

実装済みファイル:
- `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py`
- `src/auto_nav/scripts/launch_cmd_vel_bridge.py`
- `src/robot_control/src/robot_control/ros2_node.py`（nav_mode 対応を追加）

実装上の変更点（計画との差異）:
- 逆運動学の符号は m3508.py との整合を取り、以下の式を採用:
  `v_fl = +Vx+Vy-G*ω`, `v_fr = -Vx+Vy-G*ω`, `v_rl = +Vx-Vy-G*ω`, `v_rr = -Vx-Vy-G*ω`
- `auto → manual` 切り替え時にゼロ RPM 指令を送信（急動作防止）を実装済み
- `robot_control` の `send_control_command()` は `auto` モード時にスキップするよう修正
- `mapping_launch.py` に `cmd_vel_bridge_node` を追加済み

逆運動学の数値検証: 前進・横移動・回転・複合すべてのパターンで前進運動学との整合を確認済み（誤差ゼロ）

---

### フェーズ 3: SLAM セットアップ（優先度: 高）

slam_toolbox を **mapping モード** と **localization モード** の2段階で使う。

#### 設定ファイル①: マッピング用

**ファイル:** `src/auto_nav/config/slam_mapping_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping          # 地図を新規作成する
    scan_topic: /scan
    odom_frame: odom
    map_frame: map
    base_frame: base_link

    resolution: 0.05       # [m/cell] 5cm解像度
    max_laser_range: 10.0  # [m]
    map_update_interval: 1.0

    # ループクロージャ: 一周したときに誤差を修正
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_max_variance_coarse: 3.0
```

#### 設定ファイル②: ローカリゼーション用

**ファイル:** `src/auto_nav/config/slam_localization_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    mode: localization     # 保存した地図で自己位置推定のみ行う
    scan_topic: /scan
    odom_frame: odom
    map_frame: map
    base_frame: base_link

    max_laser_range: 10.0
    map_file_name: ""      # ローンチ引数で上書き
    map_start_at_dock: true
```

#### 地図の保存と読み込み

```bash
# マッピング完了後に保存（試合ごとに上書きでOK）
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/taiga/maps/field'}"

# または nav2_map_server を使う場合
ros2 run nav2_map_server map_saver_cli -f ~/maps/field
# → field.pgm (画像) と field.yaml (メタデータ) が生成される
```

#### 実機動作確認（フェーズ3）

**目的:** LiDAR スキャンと走行データから地図が正しく生成されること、
ローカリゼーションモードで自己位置推定が安定することを確認する。

**ステップA: マッピング動作確認**

```bash
ros2 launch auto_nav mapping_launch.py
# 別端末で RViz2 を起動
rviz2
# RViz2 の Add から以下を追加:
#   - Map (/map)
#   - LaserScan (/scan)
#   - Odometry (/odom)
#   - TF
```

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| スキャン表示 | 起動直後 | RViz2 で `/scan` の点群が正しく見える |
| 地図構築 | ゆっくり走行（最大0.3m/s）して部屋を一周 | `/map` に壁が黒線で描画される |
| ループクロージャ | 一周してスタート地点に戻る | 地図の壁がつながり、ズレが補正される |

**よくある問題:**
- 地図が歪む → 走行速度が速すぎる。0.2m/s 以下で走行する
- `/scan` が表示されない → `laser_frame` の TF が配信されているか確認（`ros2 run tf2_tools view_frames`）
- ループクロージャが起きない → 部屋が広すぎて特徴点が少ない場合は `loop_match_minimum_chain_size` を下げる

**ステップB: ローカリゼーション動作確認**

```bash
# 地図を保存してからローカリゼーションモードで再起動
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/taiga/maps/field'}"
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field.yaml
```

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| 初期位置推定 | 起動後、ロボットを動かさずに待つ | RViz2 でロボット位置（矢印）が実際の場所付近に表示される |
| 走行後の追従 | ゆっくり走行する | RViz2 のロボット位置が実際と一致したまま動く |
| 誤差確認 | 1m走行後に実際の移動量と比較 | ズレが10cm以内であること |

---

### フェーズ 4: ウェイポイント管理（優先度: 高）

フィールドの寸法誤差を吸収するため、ウェイポイントは**固定値をハードコードせず**、
マッピング後に地図座標系で定義・保存する仕組みにする。

**ファイル:** `src/auto_nav/config/waypoints.yaml`

```yaml
# 試合前マッピング後、実際の地図を見ながら座標を調整して保存する
# 座標系: map フレーム (マッピング開始点が原点)
waypoints:
  start:
    x: 0.0
    y: 0.0
    theta: 0.0
  goal_A:
    x: 2.1       # ← マッピング後に RViz2 で実測して記入
    y: 0.5
    theta: 1.57
  goal_B:
    x: 1.5
    y: -0.8
    theta: 0.0
```

**運用手順:**
1. マッピング完了後、RViz2 で地図を表示
2. 各ウェイポイント位置をクリックして座標を読み取る
3. `waypoints.yaml` に記入して保存
4. ローカリゼーションモードでノードを起動

#### 実機動作確認（フェーズ4）

**目的:** ウェイポイントの座標が地図上の正しい場所に対応しているか確認する。
（ここはシミュレーションではなく、実際の地図を見ながら目視で確認する。）

**手順:**

```bash
# ローカリゼーションモードで起動
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field.yaml

# RViz2 でウェイポイントを視覚的に確認
# PoseStamped をパブリッシュして地図上に表示する
ros2 topic pub /check_waypoint geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.1, y: 0.5, z: 0.0}}}" --once
```

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| 座標目視確認 | waypoints.yaml の各座標を RViz2 に表示 | 矢印が実際の目標場所（壁・マーカーなど）に重なる |
| ロボット位置での確認 | ロボットを各ウェイポイントに手動で移動 | `/odom` の座標が waypoints.yaml の値と±10cm以内で一致 |

**座標の読み取り方（RViz2）:**
1. RViz2 メニュー → Panels → Tool Properties を開く
2. 「Publish Point」ツールを選択（またはキーボード `p`）
3. 地図上の目標地点をクリック → 下部ステータスバーに `x, y` 座標が表示される
4. その値を `waypoints.yaml` に記入する

---

### フェーズ 5: 自律走行ノード（優先度: 中）

目標ウェイポイントを受け取り、Nav2 に送信する制御ノード。

**ファイル:** `src/auto_nav/src/auto_nav/routing_node.py`

```python
# サブスクライブ:
#   bluetooth_rx (String) - Bluetoothからの指令
# パブリッシュ:
#   bluetooth_tx (String) - 走行状態フィードバック
#   /nav_mode (String) - "manual" or "auto" (cmd_vel_bridge へ通知)
# ActionClient:
#   /navigate_to_pose (nav2_msgs/NavigateToPose)

# Bluetoothコマンド例:
# {"type": "nav_mode", "mode": "auto"}          # 自律走行モードへ
# {"type": "nav_mode", "mode": "manual"}         # 手動モードへ戻す
# {"type": "nav_goal", "waypoint": "goal_A"}     # ウェイポイント名で指定
# {"type": "nav_goal", "x": 1.5, "y": 0.0, "theta": 0.0}  # 座標直接指定
```

**状態管理:**
- `MANUAL`: Bluetooth手動操作モード（デフォルト）
- `AUTO_IDLE`: 自律走行モード（待機中）
- `NAVIGATING`: ゴールへ向かって走行中
- `ARRIVED`: 目標地点に到達

#### 実機動作確認（フェーズ5）

**目的:** Bluetooth からのコマンドが routing_node に届き、Nav2 ActionClient が正しくゴールを送信することを確認する。
（この時点では Nav2 が未完成でもよい。ゴール送信の流れだけ確認する。）

```bash
# 起動
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field.yaml

# routing_node の状態トピックを監視
ros2 topic echo bluetooth_tx

# Bluetooth から操作（コントローラーから送信、または直接パブリッシュでテスト）
ros2 topic pub bluetooth_rx std_msgs/String \
  '{"data": "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"}' --once

ros2 topic pub bluetooth_rx std_msgs/String \
  '{"data": "{\"type\": \"nav_goal\", \"waypoint\": \"goal_A\"}"}' --once
```

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| モード切り替え | `nav_mode: auto` を送信 | `bluetooth_tx` に `"mode": "auto"` のフィードバックが返る |
| ゴール送信 | `nav_goal: goal_A` を送信 | `NavigateToPose` ActionServer にゴールが届く（`ros2 action list` で確認） |
| ARRIVED 検知 | ロボットがゴール付近に到達 | `bluetooth_tx` に `"status": "arrived"` が返る |
| 手動復帰 | `nav_mode: manual` を送信 | Bluetooth ジョイスティックでロボットが動かせる |

**よくある問題:**
- Bluetooth の JSON が routing_node に届かない → `bluetooth_rx` のトピック名と型を確認
- ActionServer が見つからない → Nav2 が起動しているか確認（`ros2 action list`）

---

### フェーズ 6: Nav2 ナビゲーションスタック（優先度: 中）

**ファイル:** `src/auto_nav/config/nav2_params.yaml`

```yaml
# コントローラーサーバー (DWBローカルプランナー)
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # オムニドライブ対応: X/Y両方向に速度を出せる
      max_vel_x: 0.5        # [m/s]
      max_vel_y: 0.5        # オムニなのでY方向も有効
      max_vel_theta: 1.0    # [rad/s]
      min_speed_xy: 0.0
      max_speed_xy: 0.6
      # holonomic_mode はDWBでは critics の設定で対応

# プランナーサーバー (A*ベース)
planner_server:
  ros__parameters:
    planner_plugin: "NavfnPlanner"
    tolerance: 0.3          # ゴール到達判定の許容誤差 [m]

# ローカルコストマップ (障害物回避用)
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3.0
      height: 3.0
      resolution: 0.05
      robot_radius: 0.35    # ロボット半径 [m]
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 0.5  # 障害物からの安全距離

# グローバルコストマップ (経路計画用)
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.05
      robot_radius: 0.35
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

#### 実機動作確認（フェーズ6）

**目的:** Nav2 が実際にパスを生成してロボットを目標位置まで移動させることを確認する。
ここがシステム全体の最終的な統合テストになる。

**RViz2 での可視化設定:**

```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field.yaml
rviz2
# 以下を追加:
#   - Map (/map)
#   - LaserScan (/scan)
#   - Path (/plan)           ← グローバルパス
#   - Path (/local_plan)     ← ローカルパス
#   - Costmap (/global_costmap/costmap)
#   - Costmap (/local_costmap/costmap)
#   - TF
```

**段階的な確認テスト:**

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| コストマップ表示 | 起動後に RViz2 確認 | 壁が障害物（赤）として表示される |
| 経路生成 | RViz2 の「2D Goal Pose」でゴール指定 | 青い経路線（/plan）が表示される |
| 追従走行 | ゴール指定後 | ロボットが経路に沿って走行する |
| ゴール到達 | ゴール付近まで走行後 | 自動停止して ARRIVED が返る |
| 障害物回避 | 経路上に段ボール箱などを置く | 迂回経路を生成して回避する |
| 距離精度 | 既知の距離（例: 2m先）をゴールに設定 | 誤差 ±15cm 以内で到達 |

**速度・パラメータ調整:**
- 到達精度が悪い場合 → `tolerance` を調整（デフォルト 0.3m）
- 動きがぎこちない場合 → `max_vel_x/y` を下げて安定性を優先
- 障害物を避けすぎる場合 → `inflation_radius` を小さくする

---

### フェーズ 7: ローンチファイル整備（優先度: 低）

#### マッピング用ローンチ

**ファイル:** `src/auto_nav/launch/mapping_launch.py`

```python
# 起動ノード (マッピングフェーズ用):
# 1. ydlidar_ros2_driver
# 2. odometry_node
# 3. slam_toolbox (mapping モード)
# 4. robot_control (Bluetooth手動操縦)
# 5. bt_communication
# ※ Nav2 は不要 (手動走行してマッピングするだけ)
```

#### 自律走行用ローンチ

**ファイル:** `src/auto_nav/launch/auto_nav_launch.py`

```python
# 起動ノード (試合中の自律走行用):
# 1. ydlidar_ros2_driver
# 2. odometry_node
# 3. cmd_vel_bridge_node
# 4. slam_toolbox (localization モード, map:= 引数で地図指定)
# 5. nav2_bringup
# 6. routing_node
# 7. robot_control (手動モード時の走行も担当)
# 8. bt_communication
```

#### 実機動作確認（フェーズ7）

**目的:** 競技を想定したフルシーケンスを一気通貫で動かし、運用手順として成立するか確認する。

**想定シナリオ（試合を模擬）:**

```
1. 電源投入・全ノード起動（mapping_launch.py）
2. Bluetooth でロボットに接続
3. フィールドを手動走行してマッピング
4. 地図保存コマンドを実行
5. auto_nav_launch.py に切り替えて再起動
6. Bluetooth から nav_mode: auto → nav_goal: goal_A を送信
7. ロボットが自律走行してゴールに到達
8. nav_mode: manual に切り替えて手動操作に戻る
```

| テスト | 合格基準 |
|--------|---------|
| 起動時間 | `mapping_launch.py` 起動から Bluetooth 接続まで 30秒以内 |
| マッピング品質 | フィールド一周（1〜2分）で地図の壁がすべてつながる |
| 切り替え時間 | 地図保存 → `auto_nav_launch.py` 起動まで 30秒以内 |
| 自律走行精度 | 各ウェイポイントに ±15cm 以内で到達 |
| 手動復帰 | `nav_mode: manual` 送信後、即座に Bluetooth 操縦が有効になる |
| 連続動作 | 複数ウェイポイントを順番に指示して問題なく動く |

**運用として確認しておくこと:**
- ラズパイの起動時に全ノードが自動起動するか（systemd サービス化の検討）
- マッピング完了の判断基準（オペレーターがどう判断するか）
- 競技時間内に全手順が収まるか（タイムアタック確認）

---

## TF ツリー構成

```
map
 └── odom                      ← slam_toolbox が配信
      └── base_link            ← odometry_node が配信
           └── laser_frame     ← static_tf で定義（LiDARの取付位置）
```

**static_tf の設定例（LiDARがロボット中心・高さ15cmに取り付けられている場合）:**
```yaml
# src/auto_nav/config/robot_tf.yaml
base_link_to_laser:
  frame_id: base_link
  child_frame_id: laser_frame
  translation: [0.0, 0.0, 0.15]   # x, y, z [m]
  rotation: [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
```

---

## 実装優先順位

| フェーズ | 内容 | 推奨作業順 |
|---------|------|-----------|
| 1 | オドメトリノード | 1番目 |
| 2 | cmd_vel ブリッジ | 2番目 |
| 3 | SLAM セットアップ | 3番目 |
| 4 | ウェイポイント管理 | 3番目（SLAM と並行） |
| 5 | 自律走行ノード | 4番目 |
| 6 | Nav2 設定 | 4番目（自律走行ノードと並行） |
| 7 | ローンチ整備 | 最後 |

---

## 動作確認手順

### ステップ1: オドメトリ確認
```bash
ros2 launch auto_nav mapping_launch.py
ros2 topic echo /odom
ros2 run tf2_tools view_frames  # TFツリーを確認
```

### ステップ2: マッピング確認
```bash
# RViz2 で /map, /scan を可視化しながら手動走行
rviz2 -d src/auto_nav/rviz/mapping.rviz
# フィールドを一周してループクロージャが成功するか確認
```

### ステップ3: 地図保存 → ローカリゼーション切り替え
```bash
# 地図保存
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/taiga/maps/field'}"

# ローカリゼーションモードで再起動
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field.yaml
```

### ステップ4: Nav2 + 自律走行確認
```bash
# RViz2 の "2D Goal Pose" でテスト、または Bluetooth から
{"type": "nav_mode", "mode": "auto"}
{"type": "nav_goal", "waypoint": "goal_A"}
```

---

## 必要パッケージ（apt/rosdep）

```bash
sudo apt install \
  ros-kilted-slam-toolbox \
  ros-kilted-nav2-bringup \
  ros-kilted-nav2-msgs \
  ros-kilted-dwb-core \
  ros-kilted-tf2-ros \
  ros-kilted-tf2-tools \
  ros-kilted-nav2-map-server
```

---

## リスクと注意点

1. **オドメトリ精度**
   - オムニホイールはスリップしやすく、長距離走行でドリフトが発生
   - LiDAR SLAM が補正するが、マッピング中にゆっくり走ること（スリップ防止）

2. **手動/自動モード競合**
   - `manual` 時: `RobotController` が `bluetooth_rx → wheel_control` を担当
   - `auto` 時: `CmdVelBridge` が `/cmd_vel → wheel_control` を担当
   - 両者が同時にパブリッシュしないよう、モード切り替えロジックを確実に実装する

3. **LiDAR取付位置の TF**
   - `laser_frame` の位置がずれるとコストマップと実際の障害物がずれる
   - LiDAR の取付位置・向きを実機で計測して `robot_tf.yaml` に正確に設定すること

4. **YDLiDAR のデッドゾーン**
   - 範囲: 0.1m〜12m（0.1m以内はデッドゾーン）
   - `robot_radius` を実際のロボット外形より少し大きく設定して安全マージンを確保

5. **ウェイポイントの試合ごと調整**
   - 毎試合マッピング後にウェイポイント座標を微調整する運用が発生する
   - 調整しやすいよう、RViz2 でのウェイポイント確認手順を事前に練習しておくこと

6. **Nav2 のオムニドライブ対応**
   - デフォルト設定は差動二輪想定のため、Y方向速度を有効にする設定が必要
   - DWBプランナーの `VelocityIterator` プラグインのパラメータを確認すること

7. **LiDAR にタイヤが映り込む問題（要対応）**
   - YDLiDAR はロボット上に搭載されており、スキャン平面に自分のタイヤが映る
   - タイヤをそのまま障害物としてコストマップに登録すると、ロボットが常に自分の周囲を障害物と認識してしまい、経路計画が失敗する
   - タイヤまでの距離（ロボット中心からタイヤ中心）: `sqrt(0.1725² + 0.2425²) ≈ 0.30m`

   **対策候補（フェーズ3 実装時に選択・検証すること）:**

   | 対策 | 方法 | メリット | デメリット |
   |------|------|---------|-----------|
   | A. LiDAR を高く取り付ける | タイヤより上にスキャン平面が来るよう高さ調整（ハードウェア） | 根本解決 | 取付変更が必要 |
   | B. 最小距離フィルタ | `laser_filters` で `min_range ≈ 0.35m` に設定 | 実装が簡単 | 0.35m以内の壁も無視される |
   | C. コストマップのフットプリント消去 | `footprint_clearing_enabled: true` + ポリゴンフットプリント定義 | Nav2標準機能 | 動的な移動中にラグが生じる場合あり |
   | D. レーザースキャンのフットプリントフィルタ | `laser_filters/LaserScanFootprintFilter` でロボット形状内の点を除去 | 精度が高い | `laser_filters` の設定が必要 |

   **推奨: まずAを検討、難しければDを実装**

   ```bash
   # laser_filters のインストール
   sudo apt install ros-kilted-laser-filters
   ```

   ```yaml
   # D の設定例（src/auto_nav/config/laser_filters.yaml）
   laser_filter_chain:
     - name: footprint_filter
       type: laser_filters/LaserScanFootprintFilter
       params:
         # ロボット外形（タイヤ外側を含む矩形）を定義
         footprint:
           - [0.22, 0.29]
           - [-0.22, 0.29]
           - [-0.22, -0.29]
           - [0.22, -0.29]
   ```

---

## 作業記録

### 2026-02-21

#### 完了した作業

**フェーズ1: オドメトリノード** ✅
- `src/auto_nav/` パッケージを新規作成
- `odometry_node.py`: `wheel_feedback` → `/odom` + TF(`odom → base_link`) を実装
- 前進運動学を数値検証済み（前進・横移動・回転・複合すべて誤差ゼロ）
- 実機確認済み: `/scan` 正常、`/odom` 正常、TF 正常
  - 実測 TF レート: 20.2 Hz（ESP32 からの wheel_feedback が ~20Hz のため。問題なし）

**フェーズ2: cmd_vel ブリッジノード** ✅（実装完了、実機確認は次回）
- `cmd_vel_bridge_node.py`: `/cmd_vel` → `wheel_control` 変換を実装
- `/nav_mode` ("manual"/"auto") でモード切り替え
- `auto → manual` 切り替え時にゼロ RPM 指令を送信（急停止防止）
- `robot_control/ros2_node.py` に `/nav_mode` 監視を追加（競合防止）
  - `auto` モード中は `send_control_command()` をスキップするよう修正

**ランチファイル整備**
- `mapping_launch.py` を作成（全ノード一括起動）
  - 起動ノード: micro_ros_agent / ydlidar / robot_control / bt_communication / odometry / cmd_vel_bridge
  - `serial_port` 引数でシリアルポートを変更可能（デフォルト: `/dev/ttyUSB0`）
- `bt_communication` の `bumble` 依存を `.venv` から解決する `PYTHONPATH` 設定を実装

#### 次回やること（優先順位順）

1. **フェーズ2 実機確認**（最優先）
   ```bash
   ros2 launch auto_nav mapping_launch.py
   ros2 topic pub /nav_mode std_msgs/String "data: 'auto'" --once
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once
   ```
   確認項目:
   - [ ] auto モード中に cmd_vel で車輪が動くか
   - [ ] manual モード中に cmd_vel を送っても車輪が動かないか
   - [ ] auto → manual 切り替え時に急停止しないか
   - [ ] 横移動（Vy）が正しく動くか（オムニ確認）

2. **LiDAR タイヤ映り込み問題の対策検討**（フェーズ3 着手前に決める）
   - LiDAR の取付高さを確認し、タイヤより上にスキャン平面が来るか確認
   - 来ない場合は `laser_filters` の `LaserScanFootprintFilter` を実装
   - リスクと注意点 7 を参照

3. **フェーズ3: SLAM セットアップ**
   - `slam_mapping_params.yaml` / `slam_localization_params.yaml` を作成
   - `mapping_launch.py` に slam_toolbox を追加
   - 必要パッケージのインストール: `sudo apt install ros-kilted-slam-toolbox`
