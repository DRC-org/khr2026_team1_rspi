# ROS2 + LiDAR 自律走行 実装計画

## 概要

既存のロボット制御システム（4輪メカナムホイール + YDLiDAR）に自律走行機能を追加する。
競技フィールドは毎回 ±5% の寸法誤差があるため、**試合前に毎回マッピング**を行い、
その試合の実際のフィールド形状に合わせた地図を生成してから自律走行する。

---

## 目標アーキテクチャ

```
[目標位置指定]
  └── Bluetooth (JSON: {"type": "nav_goal", "waypoint": "goal_A"})
              ↓
[routing_node]  ←── /map, slam_toolbox の自己位置
  ├── Nav2 ActionClient (NavigateToPose)
  └── /cmd_vel ──→ [cmd_vel_bridge_node] ──→ wheel_control ──→ [ESP32]

[SLAM Toolbox (online_async)] ←── /scan_filtered, /odom ──→ /map, /tf(map→odom)
[odometry_node]               ←── wheel_feedback ──→ /odom, /tf(odom→base_link)
[YDLiDAR]                     ──→ /scan → [laser_filter] ──→ /scan_filtered
```

---

## 競技運用フロー（毎試合の流れ）

```
① 試合前マッピング（手動走行）
   ロボットを Bluetooth で操縦しながらフィールドを一周
   slam_toolbox が /scan_filtered + /odom から地図を自動生成
        ↓ 約1〜2分
② 地図保存
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "filename: '/home/taiga/maps/field'"
        ↓
③ ローカリゼーションモードへ切り替え
   auto_nav_launch.py で起動（map:= で保存済み地図を指定）
        ↓
④ 試合開始 → 自律走行
   Bluetooth で "nav_mode: auto" + "waypoint: goal_A" を送信
   Nav2 が経路計画 → /cmd_vel → 車輪制御
```

**フィールド誤差への対応:**
マッピング後に実際の地図上でウェイポイント座標を確認・調整する。
±5% の寸法誤差は地図に自動的に反映されるため、事前のハードコードは不要。

---

## フェーズ別実装計画

### フェーズ 1: オドメトリノード ✅ 完了

`wheel_feedback` (WheelMessage) から 4輪メカナムの前進運動学で速度を計算し、
`/odom` (nav_msgs/Odometry) と TF (`odom → base_link`) を配信する。

**ファイル:** `src/auto_nav/src/auto_nav/odometry_node.py`

**実施結果（2026-02-21）:**

| 確認項目 | 結果 |
|----------|------|
| `/odom` 配信 | ✅ 正常 |
| TF `odom → base_link` | ✅ 正常（view_frames で確認済み） |
| TF 配信レート | 20 Hz（wheel_feedback が ESP32 から ~20Hz のため。SLAM 動作に問題なし） |

**注意:** `publish_rate` パラメータではなく wheel_feedback の受信レートで駆動する（タイマー駆動ではない）。
計画値 50Hz とは異なるが slam_toolbox の動作に問題なし。

**IMU による角速度補正:**
`wheel_feedback.lsm9ds1_values` に IMU データが含まれる場合、ジャイロ Z 軸（`gz`）を角速度として使用する。
タイヤスリップ時にエンコーダ由来の omega が誤差を持つ問題を抑制し、旋回精度を向上させる。
IMU 未接続時はエンコーダ由来の omega にフォールバックする。

注意事項:
- `gz` の符号は IMU の取付向きに依存する（Z 上向き・右手系なら正 = CCW）
- `gz` の単位は ESP32 ファームウェアに合わせること（rad/s を前提。deg/s の場合は × π/180 が必要）
- 絶対 `yaw`（地磁気融合）は屋内での磁気干渉で信頼性が低いため使用しない

---

### フェーズ 2: cmd_vel ブリッジノード ✅ 完了

`/cmd_vel` (geometry_msgs/Twist) を受け取り、逆運動学で `wheel_control` (WheelMessage) に変換する。
`/nav_mode` ("manual"/"auto") でモードを切り替え、robot_control との競合を防止する。

**ファイル:** `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py`

**実施結果（2026-02-21）:**
- manual モード: robot_control が bluetooth_rx → wheel_control を担当
- auto モード: cmd_vel_bridge が /cmd_vel → wheel_control を担当
- `auto → manual` 切り替え時にゼロ RPM 指令を送信（急停止防止）✅
- `robot_control/ros2_node.py`: auto モード中は `send_control_command()` をスキップ ✅

---

### フェーズ 3: SLAM セットアップ ✅ 完了

**設定ファイル:**
- `src/auto_nav/config/slam_mapping_params.yaml`: マッピング用（online_async モード）
- `src/auto_nav/config/slam_localization_params.yaml`: ローカリゼーション用（localization モード）

両ファイルとも `scan_topic: /scan_filtered` を使用（タイヤ映り込みフィルタ後のスキャン）。

**slam_toolbox ライフサイクル管理（重要）:**

`ros2 lifecycle set /slam_toolbox configure` はデーモン経由で "Node not found" になる。
`ros2 service call` はサービスが現れるまで自動待機するため、直接呼び出しを使う。

```bash
# configure → activate の正しいコマンド
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}'
sleep 2
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'

# 状態確認
ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState
```

`mapping_launch.py` では `TimerAction(period=3.0) + ExecuteProcess(bash -c "service call × 2")` で自動化済み。

**地図保存（重要: serialize_map を使うこと）:**

```bash
# save_map は nav2_map_saver が必要で result=255 で失敗する。serialize_map を使うこと。
mkdir -p /home/taiga/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "filename: '/home/taiga/maps/field'"
# → field.posegraph (19MB) + field.data (8.9MB) が生成される
```

**ローカリゼーション起動:**

```bash
# 注意: 拡張子なし（.posegraph は不要）
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field
```

**実施結果（2026-02-23）:**
- マップ生成・保存済み: 1149×540 cells（57m×27m）、解像度 0.05m/cell
- TF ツリー完全: `map(12Hz) → odom(20Hz) → base_link → laser_frame(static)` ✅

**LiDAR タイヤ映り込みフィルタ:**
`LaserScanRangeFilter` で 0.33m 未満を除去（タイヤ中心距離 ~0.30m + マージン）。
`/scan` → `/scan_filtered` として配信。

注意点:
- `LaserScanFootprintFilter` は TF 変換でサイレント失敗するため不使用
- `laser_filter_node` に `name=` を設定するとパラメータ読み込み失敗 → `name=` を削除してデフォルト名を使う

---

### フェーズ 4: ウェイポイント管理（未着手）

フィールドの寸法誤差を吸収するため、ウェイポイントは**マッピング後に地図座標系で定義**する。

**ファイル:** `src/auto_nav/config/waypoints.yaml`

```yaml
# 試合前マッピング後、実際の地図を見ながら座標を調整して保存する
# 座標系: map フレーム（マッピング開始点が原点）
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
2. RViz2 で「Publish Point」ツール（キー `p`）を選択し、目標地点をクリック → 座標が表示される
3. `waypoints.yaml` に記入して保存
4. `auto_nav_launch.py` でローカリゼーションモード起動

**座標確認手順:**

```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field
# RViz2 で PoseStamped をパブリッシュして地図上の位置を目視確認
ros2 topic pub /check_waypoint geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.1, y: 0.5, z: 0.0}}}" --once
```

| テスト | 合格基準 |
|--------|---------|
| 座標目視確認 | 矢印が実際の目標場所（壁・マーカーなど）に重なる |
| ロボット位置での確認 | ロボットを各ウェイポイントに手動移動後、/odom の座標が ±10cm 以内で一致 |

---

### フェーズ 5: 自律走行ノード ✅ 完了（2026-02-27）

Bluetooth コマンドを受け取り、Nav2 ActionClient でゴールを送信する制御ノード。

**ファイル:**
- `src/auto_nav/src/auto_nav/routing_node.py`
- `src/auto_nav/scripts/launch_routing.py`

**Bluetooth コマンド仕様:**
```json
{"type": "nav_mode", "mode": "auto"}           // 自律走行モードへ
{"type": "nav_mode", "mode": "manual"}          // 手動モードへ（走行中はゴールキャンセル）
{"type": "nav_goal", "waypoint": "waypoint_1"}  // ウェイポイント名でゴール指定
{"type": "nav_goal", "x": 1.5, "y": 0.0, "theta": 0.0}  // 直接座標でゴール指定
```

**bluetooth_tx フィードバック:**
```json
{"nav_status": "mode",       "mode": "auto"}
{"nav_status": "navigating", "waypoint": "waypoint_1"}
{"nav_status": "arrived",    "waypoint": "waypoint_1"}
{"nav_status": "cancelled"}
{"nav_status": "error",      "message": "..."}
```

**状態遷移:**
```
MANUAL ──nav_mode:auto──→ AUTO_IDLE ──nav_goal──→ NAVIGATING
  ↑                          ↑                        │
  └──nav_mode:manual─────────┘◄──────────────────────┤
                             └────────── ARRIVED ◄───┘
```

**動作確認コマンド:**

```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field transport:=udp4

# 別ターミナルで監視
ros2 topic echo /bluetooth_tx
ros2 topic echo /nav_mode

# テスト送信
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"' --once
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"nav_goal\", \"waypoint\": \"waypoint_1\"}"' --once
```

**動作確認結果（2026-02-27 実機）:**

| テスト | 結果 |
|--------|------|
| `nav_mode: auto` 送信 | ✅ bluetooth_tx に `{"nav_status":"mode","mode":"auto"}` が返る |
| `nav_goal: waypoint_5` 送信 | ✅ ロボットが走行開始 |
| 到達後 | ✅ bluetooth_tx に `{"nav_status":"arrived","waypoint":"waypoint_5"}` が返る |
| `nav_mode: manual` 送信 | ✅ Bluetooth 手動操縦に戻る |

**注意: DWB チューニングが必要**
現状ふらつきながら走行するため、以下の調整を予定:
- `max_vel_x`, `max_vel_y` の速度制限
- `xy_goal_tolerance`, `yaw_goal_tolerance` の目標許容誤差
- DWB critic の重みパラメータ（nav2_params.yaml 参照）

---

### フェーズ 6: Nav2 ナビゲーションスタック ✅ 実装完了（実走行テスト未実施）

**設定ファイル:** `src/auto_nav/config/nav2_params.yaml`

主な設定:
- プランナー: NavfnPlanner (A*: `use_astar: true`, tolerance: 0.5m)
- コントローラー: DWBLocalPlanner（メカナム対応: `max_vel_y: 0.5`, `vy_samples: 10`）
- フットプリント: `[[0.22, 0.29], [-0.22, 0.29], [-0.22, -0.29], [0.22, -0.29]]`（計算値。要実測調整）
- コストマップ: `scan_topic: /scan_filtered`, `inflation_radius: 0.55m`, resolution: 0.05m

**調整ポイント:**
1. `footprint`: 現在は計算値（L_X+0.05m × L_Y+0.05m）。実機外形を実測して調整すること
2. `inflation_radius: 0.55m`: 壁衝突が多ければ増やす。障害物を過剰回避するなら減らす
3. `RotateToGoal` critic: メカナムでは不要な場合がある。到達精度が悪い場合は scale を 0 に設定

**実機確認（2026-02-25）:**

| 確認項目 | 結果 |
|----------|------|
| 全 Nav2 ノード lifecycle | ✅ active（8ノード確認） |
| `/navigate_to_pose` ActionServer | ✅ 存在 |
| `/follow_waypoints` ActionServer | ✅ 存在 |
| global_costmap | ✅ 1149×540 cells（slam_toolbox マップと一致） |
| local_costmap | ✅ 配信中 |

**実走行テスト（未実施）:**

```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field
rviz2
# 追加: Map(/map), LaserScan(/scan_filtered), Path(/plan), Path(/local_plan),
#        Costmap(/global_costmap/costmap), Costmap(/local_costmap/costmap), TF
```

| テスト | 手順 | 合格基準 |
|--------|------|---------|
| コストマップ表示 | RViz2 で確認 | 壁が障害物（赤）として表示される |
| 経路生成 | RViz2 の「2D Goal Pose」でゴール指定 | 青い経路線 (/plan) が表示される |
| 追従走行 | ゴール指定後 | ロボットが経路に沿って走行する |
| ゴール到達 | ゴール付近まで走行後 | 自動停止 |
| 障害物回避 | 経路上に段ボール箱を置く | 迂回経路を生成して回避する |
| 距離精度 | 2m先をゴールに設定 | 誤差 ±15cm 以内で到達 |

**ノード動作確認コマンド:**

```bash
for node in controller_server planner_server behavior_server bt_navigator \
    velocity_smoother collision_monitor waypoint_follower docking_server; do
  echo -n "$node: "
  ros2 service call /${node}/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -o "label='[^']*'" | head -1
done
# 期待: 全ノード label='active'
```

---

### フェーズ 7: ローンチファイル整備

#### マッピング用ローンチ ✅ 完了

**ファイル:** `src/auto_nav/launch/mapping_launch.py`

起動ノード:
- micro_ros_agent / ydlidar_ros2_driver / laser_filter_node（/scan_filtered）
- static_tf（base_link → laser_frame, z=0.15m ※実機実測で調整すること）
- odometry_node / cmd_vel_bridge_node
- slam_toolbox（mapping モード、TimerAction で lifecycle 自動管理）
- robot_control / bt_communication

起動コマンド:
```bash
ros2 launch auto_nav mapping_launch.py
# ESP32 が /dev/ttyUSB1 の場合: serial_port:=/dev/ttyUSB1
```

#### 自律走行用ローンチ（一部完了）

**ファイル:** `src/auto_nav/launch/auto_nav_launch.py`

| ノード | 状態 |
|--------|------|
| ydlidar + laser_filter + static_tf | ✅ |
| odometry_node + cmd_vel_bridge | ✅ |
| slam_toolbox (localization モード) | ✅ |
| Nav2 (nav2_bringup/navigation_launch.py 経由) | ✅ |
| routing_node | ❌ 未実装（フェーズ5 完了後に追加） |
| robot_control + bt_communication | ✅ |

起動コマンド:
```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/taiga/maps/field
```

#### 最終統合テスト（フェーズ 5 完了後）

```
1. mapping_launch.py 起動 → Bluetooth 接続
2. フィールドを手動走行してマッピング
3. 地図保存（serialize_map）
4. auto_nav_launch.py に切り替えて再起動
5. Bluetooth から nav_mode: auto → nav_goal: goal_A を送信
6. ロボットが自律走行してゴールに到達
7. nav_mode: manual に切り替えて手動操作に戻る
```

| テスト | 合格基準 |
|--------|---------|
| 起動時間 | mapping_launch.py 起動から Bluetooth 接続まで 30秒以内 |
| マッピング品質 | フィールド一周（1〜2分）で地図の壁がすべてつながる |
| 切り替え時間 | 地図保存 → auto_nav_launch.py 起動まで 30秒以内 |
| 自律走行精度 | 各ウェイポイントに ±15cm 以内で到達 |
| 手動復帰 | nav_mode: manual 送信後、即座に Bluetooth 操縦が有効になる |

**運用として事前に確認しておくこと:**
- ラズパイ起動時の全ノード自動起動（systemd サービス化の検討）
- マッピング完了の判断基準（オペレーターがどう判断するか）
- 競技時間内に全手順が収まるか（タイムアタック確認）

---

## TF ツリー構成

```
map
 └── odom                      ← slam_toolbox が配信（mapping/localization 共通）
      └── base_link            ← odometry_node が配信（~20Hz）
           └── laser_frame     ← static_tf（z=0.15m。実機実測で調整すること）
```

---

## リスクと注意点

1. **オドメトリ精度**: メカナムホイールはスリップしやすい。マッピング中は 0.2m/s 以下で走行すること。
2. **手動/自動モード競合**: manual 時は robot_control が、auto 時は cmd_vel_bridge が wheel_control をパブリッシュする。両方が同時にパブリッシュしないようモード切り替えロジック実装済み。
3. **LiDAR 取付位置の TF**: `laser_frame` のズレはコストマップ精度に直結する。取付位置・向きを実機で計測して確認すること（現在は z=0.15m を仮設定中）。
4. **YDLiDAR のデッドゾーン**: 0.1m 以内はデッドゾーン。フットプリントにマージンを持たせること。
5. **ウェイポイントの試合ごと調整**: 毎試合マッピング後に RViz2 でウェイポイント座標を確認・調整する。RViz2 での座標読み取り操作を事前に練習しておくこと。
6. **Nav2 のメカナムホイール対応**: DWB で Y 方向速度を有効化済み（nav2_params.yaml の max_vel_y, vy_samples 参照）。
7. **LiDAR タイヤ映り込み**: `LaserScanRangeFilter` で 0.33m 未満を除去して対処済み（`/scan_filtered`）。

---

## 必要パッケージ

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

---

## 作業記録

### 2026-02-21

**フェーズ1: オドメトリノード** ✅
- `src/auto_nav/` パッケージを新規作成
- `odometry_node.py`: wheel_feedback → /odom + TF(odom→base_link) を実装
- 前進運動学を数値検証済み（前進・横移動・回転・複合すべて誤差ゼロ）
- 実機確認: /scan, /odom, TF 正常（TF 実測レート: 20.2Hz）

**フェーズ2: cmd_vel ブリッジノード** ✅
- `cmd_vel_bridge_node.py`: /cmd_vel → wheel_control 変換を実装
- /nav_mode ("manual"/"auto") でモード切り替え
- auto→manual 切り替え時ゼロ RPM 送信（急停止防止）
- `robot_control/ros2_node.py`: auto モード中は `send_control_command()` をスキップ

**ランチファイル整備**
- `mapping_launch.py` 作成（serial_port 引数でポート変更可能）
- `bt_communication` の bumble 依存: `.venv` から解決する PYTHONPATH 設定を実装

---

### 2026-02-22

**フェーズ2 実機確認** ✅
- /odom 配信レート: 20Hz、manual/auto モード切り替え正常動作確認済み
- 不具合対応: `scripts/launch_*.py` に実行権限がなかった（`chmod +x` で修正）
  → 今後スクリプトを追加する際は忘れずに実行権限を付けること

**LiDAR タイヤ映り込みフィルタ実装** ✅
- `LaserScanRangeFilter` で 0.33m 未満を除去（/scan → /scan_filtered）
- static_tf (base_link→laser_frame, z=0.15m) 追加
- 実装時の知見:
  - `LaserScanFootprintFilter` は TF 変換でサイレント失敗するため不使用
  - `laser_filter_node` に `name=` を設定するとパラメータ読み込み失敗 → `name=` 削除

変更ファイル: `laser_filters.yaml`（新規）、`mapping_launch.py`、`CMakeLists.txt`、`package.xml`

**フェーズ3: SLAM セットアップ** ✅（設定・ビルドまで完了）
- `slam_mapping_params.yaml` / `slam_localization_params.yaml` 作成
- `mapping_launch.py` に `async_slam_toolbox_node` を追加

**slam_toolbox ライフサイクル問題の解決**
- 問題: `ros2 lifecycle set /slam_toolbox configure` がデーモン経由で "Node not found"
- 根本原因: ROS2 Jazzy の slam_toolbox はライフサイクルノードだが、デーモンが正常に検出できない
- 解決: `ros2 service call /slam_toolbox/change_state` でサービスを直接呼び出す
  - `ros2 service call` はサービス出現まで自動待機するためリトライ不要
  - `mapping_launch.py`: `TimerAction(3s) + ExecuteProcess` で自動化済み

変更ファイル: `mapping_launch.py`、`slam_mapping_params.yaml`（use_lifecycle_manager: false 削除）

**Hand feedback スパム警告の修正**
- hwmc（ハンド ESP32）未接続時に 100ms ごとに WARN を出力していた
- 修正: `_hand_fb_warned` フラグで初回のみ警告。ハンド未接続でも wheel_fb は Bluetooth に送信するよう変更

---

### 2026-02-23

**コードレビュー修正（フェーズ3 全ファイル）**

| ファイル | 修正内容 |
|---------|---------|
| `slam_mapping_params.yaml` | `enable_interactive_mode: true→false`（ヘッドレス RPi で不要） |
| `slam_localization_params.yaml` | `do_loop_closing: true→false`（ローカリゼーション中は地図更新不要） |
| `odometry_node.py` | theta 積分順序修正（theta_old を先保存し位置積分に使用）、共分散行列設定 |
| `auto_nav_launch.py` | ローカリゼーション用ランチファイルのスタブを作成（map:= 引数対応） |

**フェーズ3 実走行・マップ生成・保存** ✅
- Bluetooth 手動走行でマップ生成成功
- マップサイズ: 1149×540 cells（57m×27m）、解像度 0.05m/cell
- TF ツリー完全確認: map(12Hz)→odom(20Hz)→base_link→laser_frame(static) ✅
- 地図保存: `serialize_map` で `field.posegraph` (19MB) + `field.data` (8.9MB) 生成

地図保存で判明した注意点:
- `save_map` は nav2_map_saver が必要で **result=255 で失敗する**
- 正しいコマンドは `serialize_map`（slam_toolbox ネイティブ形式）を使うこと

---

### 2026-02-26

**laser_frame TF の修正** ✅
- 位置引数（`x y z yaw pitch roll`）の順序バグを名前付き引数（`--yaw`, `--pitch` 等）に変更して修正
- LiDAR 取付角ずれ: `--yaw 1.5708`（0度方向がロボット左向き）
- LiDAR 上下反転: `--pitch 3.14159`（上下反転によるY軸ミラーを補正）
- 変更ファイル: `mapping_launch.py`, `auto_nav_launch.py`

**IMU による角速度補正** ✅
- `odometry_node.py`: `wheel_feedback.lsm9ds1_values` が存在する場合、`gz`（ジャイロ Z 軸）を omega として使用
- タイヤスリップ時のエンコーダ誤差を抑制し、旋回精度を向上
- IMU 未接続時はエンコーダ由来の omega にフォールバック
- 注意: `gz` の符号・単位（rad/s か deg/s か）は ESP32 ファームウェアで確認すること

---

### 2026-02-25

**フェーズ6 Step 1: nav2_params.yaml 作成** ✅
- `src/auto_nav/config/nav2_params.yaml` を新規作成
- map_server は slam_toolbox が /map を配信済みのため含めない
- DWB メカナム対応: `min_vel_y: -0.5`, `max_vel_y: 0.5`, `vy_samples: 10`

**フェーズ6 Step 2: auto_nav_launch.py へ Nav2 組み込み** ✅
- `nav2_bringup/launch/navigation_launch.py` を IncludeLaunchDescription で取り込み
- `package.xml` に `nav2_bringup` 依存追加
- slam_toolbox が /map を先に担当するため `bringup_launch.py`（map_server 含む）は不使用
- global_costmap の static_layer が /map 出現を待機するため追加の遅延タイマーは不要

**動作確認済み（2026-02-25 実機）** ✅: 全 Nav2 ノード active、ActionServer 確認済み

**ROS2 Jazzy / Nav2 1.x 固有のトラブル（解決済み）:**

1. `navigation_launch.py` は 10ノードを管理（旧版より多い）:
   `controller_server`, `smoother_server`, `planner_server`, `route_server`,
   `behavior_server`, `velocity_smoother`, `collision_monitor`, `bt_navigator`,
   `waypoint_follower`, `docking_server`
   → nav2_params.yaml に全ノードのパラメータを定義する必要がある
2. `bt_navigator` の `plugin_lib_names` を明示すると BT ノード ID が二重登録されて FATAL
   → `plugin_lib_names` を削除（ビルトインプラグインは自動ロードされる）
3. `collision_monitor`: `observation_sources` 未定義で configure 失敗 → params 追加で解決
4. `docking_server`: `dock_plugins` 未定義で configure 失敗 → params 追加で解決

---

### 2026-02-27

**EKF センサフュージョン実装** ✅

タイヤスリップで odom→base_link TF が崩れる問題に対し、robot_localization の ekf_node を導入。
エンコーダ（/odom_raw）と IMU ジャイロ（/imu）を EKF で融合し、局所オドメトリの精度を向上。

**アーキテクチャ変更:**
- `odometry_node.py`: IMU 融合・TF 配信を除去。トピック名を `/odom` → `/odom_raw` に変更（エンコーダのみ）
- `imu_publisher_node.py`（新規）: `wheel_feedback` から LSM9DS1 gyro/accel を取り出し `/imu` に配信
- `ekf_params.yaml`（新規）: odom_raw（vx, vy, vyaw）+ imu（gz）を融合。TF(odom→base_link) は EKF が配信
- `mapping_launch.py` / `auto_nav_launch.py`: `imu_publisher_node` + `ekf_node` を追加
- `package.xml`: `sensor_msgs`, `robot_localization` 依存追加

**発見・修正した問題:**

1. `ekf_node` のデフォルト出力トピックは `odometry/filtered`（`/odom` ではない）
   → launch ファイルで `remappings=[("odometry/filtered", "/odom")]` を追加して解決

2. 静止時に IMU gz バイアスが EKF に積分されて yaw が 0.23°/秒でドリフト
   → `imu_publisher_node.py` に静止検出を追加
   - エンコーダ omega が 0.01 rad/s 未満のとき gz 共分散を 0.01 → 10.0 にして EKF に無視させる
   - 修正後: 静止時ドリフトが 0.23°/秒 → 0.001°/秒（230倍改善）✅

3. `Write` ツールで作成したスクリプトに実行権限がつかない
   → `chmod +x scripts/launch_imu_publisher.py` で解決（再ビルド不要）

**動作確認結果（2026-02-27 実機）:**

| 確認項目 | 結果 |
|---------|------|
| /odom_raw レート | ~35Hz（ESP32 送信レート） |
| /imu レート | ~18Hz |
| /odom（EKF 出力）レート | ~17Hz |
| TF: odom→base_link（EKF） | 17.6Hz ✅ |
| TF: map→odom（slam_toolbox） | 9.5Hz ✅ |
| 静止時 yaw ドリフト | 0.01°/5秒 ✅ |
| 360° 旋回後帰還誤差 | ~14°（slam_toolbox が大局補正） |

**TF ツリー:**
```
map(9.5Hz) → odom → base_link(17.6Hz) → laser_frame(static)
                 ↑ ekf_filter_node（robot_localization）
```

**SLAM マッピングパラメータ調整** ✅

対角壁の重複描画・初期位置付近の二重壁を修正。

| パラメータ | 旧値 | 新値 | 理由 |
|-----------|------|------|------|
| `link_match_minimum_response_fine` | 0.1 | **0.4** | 対角壁でスコアが低いフレームを棄却し重複描画を防止 |
| `loop_match_minimum_response_fine` | 0.45 | **0.55** | 低品質ループクロージャを棄却し補正ジャンプ幅を縮小 |
| `loop_match_maximum_variance_coarse` | 3.0 | **1.5** | 粗マッチングの分散上限を下げ、偽ループ候補を除外 |

**初期位置付近の二重壁の根本原因:**
ループクロージャ発動時にポーズグラフは補正されるが、補正前に書き込んだ占有グリッドの観測値は slam_toolbox のオンラインモードでは消去されない仕様のため、補正前後の壁が両方残る。ループクロージャの品質を上げることで補正量を最小化し、壁のずれを実用上無視できる程度に抑制した。

**調整ガイド:**
- 対角壁がまだ重複 → `link_match_minimum_response_fine` を 0.45 まで上げる
- 地図が疎になる（壁が薄い）→ 0.35 まで下げる
- ループクロージャが発動しない（地図が曲がったまま）→ `loop_match_minimum_response_fine` を 0.5 まで下げる

**マッピング完了確認（2026-02-27）:**
- 対角壁：1本線でシャープに描画 ✅
- 初期位置付近の二重壁：解消 ✅
- 地図保存: `field.posegraph`（5.0MB）+ `field.data`（164KB）✅
