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
     "filename: '/home/pi/maps/field'"
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

**地図保存（用途で使い分け）:**

```bash
# ※ 以下どちらも mapping_launch.py 起動中（slam_toolbox が動いている間）に実行すること

# [1] slam_toolbox ネイティブ形式（マッピング再開・精度保持用）
mkdir -p /home/pi/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "filename: '/home/pi/maps/field'"
# → field.posegraph + field.data が生成される

# [2] Nav2 / generate_waypoints.py 用 pgm/yaml 形式
# map_subscribe_transient_local:=true 必須（slam_toolbox は TRANSIENT_LOCAL で /map を配信）
ros2 run nav2_map_server map_saver_cli -f /home/pi/maps/field \
  --ros-args -p use_sim_time:=false -p map_subscribe_transient_local:=true
# → field.pgm + field.yaml が生成される
```

**ローカリゼーション起動:**

```bash
# 注意: 拡張子なし（.posegraph は不要）
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field
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
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field
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
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4

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
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field
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
| routing_node | ✅ |
| robot_control + bt_communication | ✅ |

起動コマンド:
```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field
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

### フェーズ 8: Web コントロールシステム（未着手）

**背景:** 競技規則により試合中は一切の外部通信が禁止。Bluetooth コントローラは自動制御失敗時の手動フォールバック専用とし、通常運用は WebSocket 経由に移行する。

**基本方針:** UI は既存の `khr2026_team1_bt_controller`（React + TypeScript + Vite）リポジトリに **WebSocket 接続ページを追加**する。新規に HTML を書き起こすのではなく、既存の UI コンポーネント・スタイル・プロトコルを流用する。

```
[khr2026_team1_bt_controller (ビルド済み SPA)]
        ↕ WebSocket (ws://<raspi-ip>:8080/ws)
[web_server_node（本リポジトリ: web_control パッケージ）]
        ↕ bluetooth_rx / bluetooth_tx (std_msgs/String)
[routing_node / robot_control]
```

オペレーターのスマホ/タブレットはラズパイの Wi-Fi AP に接続し、
`http://<raspi-ip>:5173`（開発サーバー）または GitHub Pages の URL を開く。
ラズパイ側は WebSocket サーバーのみ起動すればよい（HTML ファイルの配信は不要）。

---

#### 8-1: RPi 側 `web_control` パッケージ（新規）

**責務:** WebSocket ブリッジのみ。HTTP 静的ファイル配信は行わない。

**ディレクトリ構成:**
```
src/web_control/
├── CMakeLists.txt
├── package.xml
├── pyproject.toml           ← uv 管理（aiohttp を追加）
├── src/web_control/
│   ├── __init__.py
│   └── web_server_node.py
└── scripts/
    └── launch_web_server.py
```

**依存ライブラリ:** `aiohttp`（uv で管理）

**パッケージ作成手順（実装時）:**
```bash
cd src
ros2 pkg create web_control
cd web_control
uv init . --lib --python-preference only-system
uv venv --system-site-packages
uv add aiohttp
```

**web_server_node.py の動作仕様:**

1. `aiohttp` で WebSocket サーバーをポート 8080 で起動（`ws://0.0.0.0:8080/ws`）
2. `GET /ws` → WebSocket ハンドラ（CORS ヘッダー付き。GitHub Pages からの接続を許可）
3. 接続中の全クライアントをセットで管理し、ブロードキャスト

**asyncio + rclpy 統合方針:**
- rclpy `MultiThreadedExecutor` でスピン（メインスレッド）
- `aiohttp` の asyncio ループは `threading.Thread` + `asyncio.new_event_loop()` で別スレッドで起動
- ROS2 コールバック → WebSocket: `asyncio.run_coroutine_threadsafe(broadcast(msg), loop)`
- WebSocket → ROS2: `publisher.publish()`（rclpy の publish はスレッドセーフ）

**ROS2 インターフェース:**
- Sub: `bluetooth_tx` (String) → WebSocket 全クライアントにブロードキャスト
- Pub: `bluetooth_rx` (String) ← WebSocket から受信したコマンドを転送

**CORS 設定:** ブラウザが GitHub Pages 等の外部ドメインから WebSocket に接続するため、
`Access-Control-Allow-Origin: *` ヘッダーを HTTP Upgrade レスポンスに付与する。

---

#### 8-2: `khr2026_team1_bt_controller` リポジトリへの追加

既存スタック（React 19 + TypeScript + Vite + Tailwind CSS 4 + React Router 7）をそのまま使用。

**既存プロトコルとの整合:**

現在の Bluetooth 送信 JSON（`bluetooth.tsx` の `sendJsonData()`）と WebSocket 送信 JSON は**完全に同一フォーマット**。
既存の `OpButton.tsx` / `useJoystickFields.tsx` 等のコンポーネントは接続手段を意識せずに再利用できる。

---

##### 追加ファイル

**`src/hooks/useWebSocketConnect.tsx`（新規）**

`useBluetoothConnect.tsx` と同等のインターフェースを提供する WebSocket 接続管理フック。

```typescript
// インターフェース（useBluetoothConnect と揃える）
interface UseWebSocketConnectReturn {
  isConnected: boolean
  connect: (url: string) => Promise<void>   // 例: "ws://192.168.4.1:8080/ws"
  disconnect: () => void
  sendJson: (data: unknown) => void         // JSON シリアライズして send
  lastMessage: unknown | null              // 最後に受信した JSON（パース済み）
}
```

**送信レート:** joystick は `useJoystickFields.tsx` の既存 `SEND_INTERVAL = 100ms` をそのまま流用。
`sendJson` 呼び出し元は Bluetooth 版と同じコンポーネントを再利用できる。

---

##### 変更ファイル

**`src/routes/Index.tsx`（変更）**

既存のメニューに「Auto Nav（WebSocket）」を追加:

| メニュー | 説明 |
|---------|------|
| Controller（既存） | ロボットを Bluetooth で手動コントロールする |
| PID Tuning（既存） | 足回りの PID ゲインを調整する |
| **Auto Nav（新規）** | **自律走行の制御・モニタリング（WebSocket 接続）** |
| **Controller (WS)（新規）** | **WebSocket 経由で手動コントロール（競技本番用）** |

**`src/routes/Controller.tsx`（変更）**

Bluetooth 接続の既存実装に加え、WebSocket 接続オプションを追加。
Index ページからの遷移パラメータ（`?mode=ws&url=ws://...`）で接続手段を切り替える。

---

##### 新規ルート: `src/routes/AutoNav/`

```
src/routes/AutoNav/
├── index.tsx           ← AutoNav ページ本体
├── useAutoNav.ts       ← WebSocket 接続 + 状態管理フック
└── CourtSelector.tsx   ← コート選択コンポーネント
```

**`AutoNav/index.tsx` の UI 仕様:**

- **接続セクション:**
  - WebSocket URL 入力欄（デフォルト: `ws://192.168.4.1:8080/ws`）
  - 「接続」「切断」ボタン・接続状態インジケータ

- **コート選択セクション:**
  - 「青コート」「赤コート」ボタン（選択中は強調表示）
  - 選択時に `{"type": "set_court", "court": "blue"|"red"}` を送信

- **制御セクション:**
  - 「自動開始」ボタン（`{"type": "start_auto"}` を送信）
  - 「停止」ボタン（`{"type": "stop_auto"}` を送信）
  - 誤タップ防止のため操作確認ダイアログを挟む

- **ステータスセクション:**
  - モード: `MANUAL` / `AUTO_IDLE` / `NAVIGATING`
  - 現在ウェイポイント名 + 進捗 `1 / 3`
  - フィールド SVG（`khr2026_field.svg` の既存アセット再利用）でロボット位置表示

- **ログセクション:**
  - `bluetooth_tx` から受信した `nav_status` メッセージを時系列表示（最新 20 件）

---

#### 8-3: WebSocket メッセージプロトコル

**ブラウザ → サーバー（送信）:**

Bluetooth 版と完全に同一フォーマット。既存の JSON 仕様をそのまま流用。

| type | フィールド | 説明 |
|------|-----------|------|
| `set_court` | `court: "blue"\|"red"` | コート選択（routing_node が受け取り座標変換に使用） |
| `start_auto` | なし | 自動シーケンス開始 |
| `stop_auto` | なし | 自動制御停止 |
| `nav_goal` | `waypoint: string` | 個別ウェイポイント指定（デバッグ用） |
| `joystick` | `l_x, l_y, r: int (-10〜10)` | 手動走行（Bluetooth 版と同一） |
| `hand_control` | `target, control_type, action` | ハンド操作（Bluetooth 版と同一） |
| `pid_gains` | `kp, ki, kd: float` | PID ゲイン調整（Bluetooth 版と同一） |

**サーバー → ブラウザ（受信）:**

`bluetooth_tx` に流れるメッセージをそのまま転送。PID チューニング画面でも WebSocket 接続を使えば Bluetooth と同様に動作する。

| 発生元 | 内容 |
|--------|------|
| routing_node | `{"nav_status": "navigating"\|"arrived"\|"cancelled"\|"error", ...}` |
| robot_control | `{"m3508_rpms": {...}, "yagura": {...}, "ring": {...}, ...}` |
| web_server_node | `{"type": "server_status", "court": "blue"\|"red"}` |

---

### フェーズ 9: 競技対応機能（未着手）

#### 9-1: コート切り替え（青/赤 Y 軸対称）

**前提:** 青コートと赤コートはフィールド中央の縦線（Y 軸）を挟んで線対称。ウェイポイントは青コートを基準に定義し、赤コートは Y 座標を自動反転する。

**重要:** マッピング時はロボットをフィールドの X 軸（前進方向）に合わせて配置すること。マッピング開始点の正面方向が Y 軸対称の基準になる。

**座標変換式:**
```
赤コート:
  x_red   =  x_blue          (X 座標はそのまま)
  y_red   = -y_blue          (Y 座標を反転)
  theta_red = -theta_blue    (向きも反転：右向き ↔ 左向き)
```

**実装場所:** `routing_node.py` に `_current_court: str = "blue"` を追加。
`_handle_goal()` 内でゴール送信前に座標変換を適用。
`set_court` コマンドは `bluetooth_rx` に `{"type": "set_court", "court": "blue"|"red"}` として流れ、routing_node が受け取る（`on_controller_command` には影響しない、routing_node だけが処理）。

**`on_arrive` シーケンスへの適用:** シーケンス中のハンド操作は対称性に依存しないため変換不要。ただし将来的に「向き補正のための小移動」を追加する場合は座標変換が必要。

#### 9-2: ウェイポイント到着時のハンドシーケンス

**`waypoints.yaml` 拡張フォーマット:**

```yaml
waypoints:
  waypoint_1:
    x: -0.677
    y: 1.275
    theta: 0.0
    on_arrive:
      - action: hand_control
        target: yagura_1
        control_type: pos
        action_value: up
      - action: wait
        duration: 1.5        # 秒
      - action: hand_control
        target: ring_1
        control_type: state
        action_value: open
      - action: wait
        duration: 0.5
      - action: hand_control
        target: ring_1
        control_type: state
        action_value: close
  waypoint_2:
    x: -1.859
    y: 2.419
    theta: 0.0
    on_arrive: []            # 到着後は何もしない
```

**`on_arrive` で使える action 種別:**

| action | 必須フィールド | 説明 |
|--------|-------------|------|
| `hand_control` | `target, control_type, action_value` | `bluetooth_rx` に `{"type": "hand_control", ...}` を発行 |
| `wait` | `duration` (秒) | 指定秒数待機（rclpy Timer or time.sleep をスレッドで） |

**`target` の値:** `yagura_1`, `yagura_2`, `ring_1`, `ring_2`
**`control_type` の値:** `pos`, `state`
**`action_value` の値（yagura pos）:** `up`, `down`, `stopped`
**`action_value` の値（yagura state）:** `open`, `close`, `stopped`
**`action_value` の値（ring pos）:** `pickup`, `yagura`, `honmaru`, `stopped`
**`action_value` の値（ring state）:** `open`, `close`, `stopped`

**実行フロー（routing_node.py の変更点）:**
```
Nav2 SUCCEEDED 受信 (_result_cb)
  → on_arrive リストを別スレッド（または Timer チェーン）で順次実行
      - hand_control: bluetooth_rx に JSON を publish
      - wait: time.sleep（別スレッドなら OK）
  → 全ステップ完了後に bluetooth_tx に "arrived" を発行
  → _state = "AUTO_IDLE"（または次の waypoint へ）
```

**中断処理:** シーケンス実行中に `nav_mode: manual` を受信したら即座に中断（フラグで制御）。`"cancelled"` を発行。

**実装の注意:** `time.sleep` を rclpy のメインスレッドで呼ぶと他のコールバックがブロックされる。専用スレッドで実行するか、rclpy の `Timer` を使ったコールバックチェーンで実装すること。

#### 9-3: 自動シーケンス全体フロー

**`auto_sequence` を `waypoints.yaml` に追加:**

```yaml
auto_sequence:
  - waypoint_1
  - waypoint_2
  - waypoint_3

waypoints:
  waypoint_1: ...
  waypoint_2: ...
  waypoint_3: ...
```

**`start_auto` コマンドの動作:**
1. Web UI の「自動開始」→ `{"type": "start_auto"}` → web_server_node → `bluetooth_rx`
2. routing_node が受信 → `_state` を `NAVIGATING` に設定
3. `auto_sequence` の先頭から順に:
   a. `_send_goal(x, y, theta)` を呼び出し
   b. `bluetooth_tx` に `{"nav_status": "navigating", "waypoint": "waypoint_X", "progress": "1/3"}` を発行
   c. Nav2 SUCCEEDED → `on_arrive` 実行 → 完了待ち
   d. `bluetooth_tx` に `{"nav_status": "arrived", "waypoint": "waypoint_X", "progress": "1/3"}` を発行
   e. 次の waypoint へ
4. 全 waypoint 完了 → `bluetooth_tx` に `{"nav_status": "completed"}` を発行 → `_state = "AUTO_IDLE"`

**`stop_auto` / `nav_mode: manual` の挙動:** 実行中のゴールをキャンセル + シーケンスを中断

---

### フェーズ 10: 壁基準ウェイポイント生成（未着手）

**課題:** テストランでは RViz2 を確認してウェイポイント座標を記入する時間が取れない。

**解決策:** ウェイポイントを「南壁から 50cm、西壁から 100cm」のような相対座標で事前定義し、マッピング後にスクリプトで自動変換する。±5% の寸法誤差も PGM に自動反映されるため、変換後の座標は実フィールドに対応する。

#### 10-1: waypoints_relative.yaml（新規）

**ファイルパス:** `src/auto_nav/config/waypoints_relative.yaml`

```yaml
# 競技前に事前定義するウェイポイント（壁からの相対距離）
# 方角の定義:
#   north: マッピング開始時にロボットが向いていた方向の壁
#   south: その反対の壁
#   east:  ロボットの右側の壁（マッピング開始時）
#   west:  ロボットの左側の壁（マッピング開始時）
# distance: その壁からの距離 [m]（壁面から内側へ）
# cross_wall / cross_distance: 垂直方向の基準壁と距離

auto_sequence:
  - waypoint_1
  - waypoint_2

waypoints:
  waypoint_1:
    from_wall: south
    distance: 0.5
    cross_wall: west
    cross_distance: 1.0
    theta: 1.57       # 北を向く
    on_arrive:
      - action: hand_control
        target: ring_1
        control_type: state
        action_value: open
  waypoint_2:
    from_wall: north
    distance: 0.3
    cross_wall: west
    cross_distance: 2.0
    theta: 0.0
    on_arrive: []
```

#### 10-2: generate_waypoints.py（新規スクリプト）

**ファイルパス:** `src/auto_nav/scripts/generate_waypoints.py`

**使い方:**
```bash
python3 src/auto_nav/scripts/generate_waypoints.py \
  --map /home/pi/maps/field.pgm \
  --meta /home/pi/maps/field.yaml \
  --relative src/auto_nav/config/waypoints_relative.yaml \
  --output src/auto_nav/config/waypoints.yaml
```

**依存ライブラリ:** `Pillow`（PIL）のみ（OpenCV 不使用で軽量に）

**処理フロー:**
1. `field.pgm` を PIL で読み込む
2. `field.yaml` から `resolution`（m/cell）と `origin`（マップ原点のワールド座標）を取得
3. 占有グリッド（0=空き, 100=占有, -1=未探索）から壁領域を検出:
   - 占有値 ≥ 65 をピクセルで壁と判定（slam_toolbox のデフォルト設定に合わせた閾値）
   - 各方向の壁の外縁座標を検出（各行/列の中央値フィルタでノイズ除去）
   - 結果: `wall_north_y`, `wall_south_y`, `wall_east_x`, `wall_west_x`（ワールド座標 [m]）
4. 相対定義を絶対座標に変換:
   ```
   from_wall=south, distance=0.5 → y_abs = wall_south_y + 0.5
   from_wall=north, distance=0.3 → y_abs = wall_north_y - 0.3
   cross_wall=west,  cross_distance=1.0 → x_abs = wall_west_x + 1.0
   cross_wall=east,  cross_distance=2.0 → x_abs = wall_east_x - 2.0
   ```
5. 変換結果（絶対座標 x, y, theta + on_arrive）を `waypoints.yaml` に書き出し
6. 生成された `waypoints.yaml` は既存フォーマットと互換（routing_node がそのまま読める）

**事前準備（マッピングフローへの追記）:** `serialize_map` の後に以下も実行する:
```bash
# PGM マップファイルも保存する（generate_waypoints.py が必要とする）
# ※ mapping_launch.py 起動中（slam_toolbox が動いている間）に実行すること
ros2 run nav2_map_server map_saver_cli \
  -f /home/pi/maps/field \
  --ros-args -p use_sim_time:=false -p map_subscribe_transient_local:=true
# → /home/pi/maps/field.pgm + /home/pi/maps/field.yaml が生成される
```

**壁検出の精度注意点:**
- ロボットがフィールドを一周して壁を全周スキャンしていることが前提（壁が途切れていると誤検出）
- 壁の厚さ（20cm 程度）の中心線ではなく外縁を基準にするため、`distance` の定義は「壁面（内側面）からの距離」とする
- 検出結果をスクリプト実行時に標準出力に表示し、目視で確認できるようにする

---

### 競技運用フロー（更新版）

**フェーズ 8〜10 完成後の全体フロー:**

```
① マッピング（試合前）
   mapping_launch.py 起動 → Bluetooth/手動で一周走行
   serialize_map → map_saver_cli（PGM も保存）

② ウェイポイント生成（マッピング直後、5秒程度）
   python3 generate_waypoints.py --map field.pgm --meta field.yaml \
     --relative waypoints_relative.yaml --output waypoints.yaml
   ※ waypoints_relative.yaml は事前に「壁から〇cm」形式で定義済み

③ 本番起動
   auto_nav_launch.py 起動（web_server_node も起動される）
   ブラウザで localhost:8080 にアクセス（またはオペレーターのデバイスから）

④ コート選択
   Web UI でコート（青/赤）を選択

⑤ 自動制御開始
   Web UI の「自動開始」ボタン → routing_node が auto_sequence を実行

⑥ フォールバック（自動制御不能時のみ）
   iPad を Bluetooth 接続 → 手動操縦モードに切り替え
```

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
5. **ウェイポイントの試合ごと調整（フェーズ 10 完成後は自動化）**: フェーズ 10 完成前は毎試合 RViz2 でウェイポイント座標を確認・調整する必要がある。フェーズ 10 完成後は `generate_waypoints.py` で自動生成。
6. **Nav2 のメカナムホイール対応**: DWB で Y 方向速度を有効化済み（nav2_params.yaml の max_vel_y, vy_samples 参照）。
7. **LiDAR タイヤ映り込み**: `LaserScanRangeFilter` で 0.33m 未満を除去して対処済み（`/scan_filtered`）。
8. **コート対称軸の前提**: 赤コートの Y 軸反転はマッピング開始時のロボット向きを X 軸としている。マッピング時は必ずフィールド中央線に正対して配置すること。
9. **asyncio + rclpy の統合**: web_server_node は aiohttp の asyncio ループと rclpy のループを別スレッドで動かす。`asyncio.run_coroutine_threadsafe` / スレッドセーフ publish 以外での跨ぎ操作は禁止。
10. **on_arrive 実行中のブロッキング**: `time.sleep` を rclpy メインスレッドで呼ぶとコールバックが止まる。on_arrive シーケンスは専用スレッドで実行すること。
11. **壁検出の前提**: `generate_waypoints.py` はロボットがフィールド全周を走行して壁が途切れていないことを前提とする。マッピング精度が低い場合は壁検出が失敗する可能性がある。

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
- `map_saver_cli` は slam_toolbox が **動いている間** に実行すること（/map の Publisher count が 0 だと失敗）
- slam_toolbox は /map を TRANSIENT_LOCAL で配信するため `map_subscribe_transient_local:=true` が必須
- `serialize_map`（slam_toolbox ネイティブ形式）は常に推奨。pgm/yaml が必要なときは上記 2 条件を満たした上で `map_saver_cli` を使う

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

---

**フェーズ5: routing_node 実装** ✅

Bluetooth コマンドを受け取り Nav2 NavigateToPose ActionServer にゴールを送信するノードを実装。

**新規作成ファイル:**
- `src/auto_nav/src/auto_nav/routing_node.py`
- `src/auto_nav/scripts/launch_routing.py`

**変更ファイル:**
- `src/auto_nav/package.xml`: `nav2_msgs` 依存追加
- `src/auto_nav/launch/auto_nav_launch.py`: routing_node 追加

**発見・解決した問題:**

1. **ActionClient + SingleThreadedExecutor でサブスクリプションコールバックが無音になる（最重要）**
   - 症状: `bluetooth_rx` は届いている（`ros2 topic echo` で確認）のに `/nav_mode` が全く publish されない
   - 根本原因: `ActionClient` は初期化時に Nav2 の `_action/status` トピックを受信するための Waitable を登録する。このトピックが高頻度で publish されるため、`SingleThreadedExecutor` のループで Waitable が占有され、bluetooth_rx コールバックが一切実行されない
   - 修正: `main()` で `rclpy.spin()` → `MultiThreadedExecutor` に変更

   ```python
   from rclpy.executors import MultiThreadedExecutor
   executor = MultiThreadedExecutor()
   executor.add_node(node)
   executor.spin()
   ```

2. **「ロボットが動かない」→ 実は既にウェイポイント近傍にいた**
   - 症状: nav_goal 送信直後に `arrived` が返る
   - 原因: `xy_goal_tolerance: 0.25m`。ロボットが waypoint_1 から 0.09m の地点にいたため Nav2 が即座に SUCCEEDED を返した
   - 対処: ロボットを別の場所に移動してから waypoint_5（~5m 先）でテスト → 正常走行確認

**FastDDS SHM エラー（無害）:**
- `[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed`
- 原因: 強制終了したプロセスが残した古い共有メモリファイル
- 影響なし（FastDDS が自動再作成）。気になる場合は `rm -f /tmp/fastrtps_port*`

**実機動作確認（2026-02-27）:**

| テスト | 結果 |
|--------|------|
| nav_mode: auto 送信 | ✅ /nav_mode に "auto"、bluetooth_tx に `{"nav_status":"mode","mode":"auto"}` |
| nav_goal: waypoint_5 送信 | ✅ ロボット走行開始 |
| 到達後 | ✅ bluetooth_tx に `{"nav_status":"arrived","waypoint":"waypoint_5"}` |
| nav_mode: manual 送信（走行中） | ✅ ゴールキャンセル、手動操縦に復帰 |

**残課題:** DWB コントローラのふらつき対策（nav2_params.yaml のチューニング）が必要

---

---

### 2026-02-27（後半）

**フェーズ 8〜10 の要件・実装方針を策定**

競技規則（試合中は外部通信禁止）を受けて、以下のフェーズを計画として追加:

- **フェーズ 8**: Web コントロールシステム（`web_control` パッケージは WebSocket ブリッジのみ。UI は `khr2026_team1_bt_controller` に WebSocket 対応ページを追加）
- **フェーズ 9**: 競技対応機能（コート切り替え Y 軸反転、waypoint `on_arrive` ハンドシーケンス、`start_auto` 全自動シーケンス）
- **フェーズ 10**: 壁基準ウェイポイント生成（`waypoints_relative.yaml` + `generate_waypoints.py`）

詳細はフェーズ別実装計画セクションに記載済み。実機到着後に順次実装予定。

---

**DWB チューニング + MIN_RPM 対応** ✅

変更ファイル:
- `src/auto_nav/config/nav2_params.yaml`
- `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py`

**nav2_params.yaml の変更点（8箇所）:**

| パラメータ | Before | After | 理由 |
|-----------|--------|-------|------|
| `general_goal_checker.xy_goal_tolerance` | 0.25 | 0.08 | ±8cm 精度目標 |
| `general_goal_checker.yaw_goal_tolerance` | 0.25 | 0.10 | 到着時の向き精度（±6°） |
| `FollowPath.max_vel_x` | 0.5 | 1.0 | 速度倍増 |
| `FollowPath.max_speed_xy` | 0.5 | 1.0 | 速度倍増 |
| `FollowPath.sim_time` | 1.7 | 1.2 | ふらつき抑制 |
| `FollowPath.xy_goal_tolerance` | 0.25 | 0.08 | RotateToGoal トリガー精度 |
| `FollowPath.PathAlign.scale` | 32.0 | 8.0 | ふらつきの主原因を除去 |
| `velocity_smoother.max_velocity` | [0.5, 0.5, 1.0] | [1.0, 0.5, 1.0] | DWB 上限に合わせる |

**cmd_vel_bridge_node.py の変更点:**
- `MIN_RPM = 700.0` を追加
- 逆運動学後の RPM が `0 < max_abs < 700` のとき、車輪比率を保ったまま 700 RPM まで底上げ
- 目的: M3508 の静止摩擦を超えられず不動になる問題を解消
- 700 RPM ≈ 0.19 m/s 相当。動きが速すぎる場合は値を下げて調整すること

**動作確認コマンド:**
```bash
# 1. 起動
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4

# 2. 起動後 5 秒待ってから auto モードに切り替え
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"}' --once

# 3. waypoint をセット（nav_goal → "navigating" 応答まで 5〜8 秒かかる場合あり）
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"nav_goal\", \"waypoint\": \"waypoint_1\"}"}' --once

# 4. bluetooth_tx で "navigating" → "arrived" を確認
ros2 topic echo /bluetooth_tx
```

**注意事項:**
- nav_goal 送信後、ロボットが動き出すまで 5〜8 秒かかる（ActionClient の wait_for_server のため）
- MIN_RPM=700 はチューニング値。速すぎる・止まれない場合は下げること

---

### 2026-03-01

**フェーズ10: 壁基準ウェイポイント自動生成** ✅

作成ファイル:
- `src/auto_nav/config/waypoints_relative.yaml`: 壁からの相対距離でウェイポイントを定義するテンプレート
- `src/auto_nav/scripts/generate_waypoints.py`: PGM マップを解析して絶対座標の waypoints.yaml を生成するスタンドアロンスクリプト

実装上の注意:
- PGM 読み込みは純粋 Python（Pillow 不要）。P2(ASCII)/P5(binary) 両対応
- 壁検出: 各列/各行で occupied ピクセルの最小・最大を取り statistics.median で外れ値除去
- 占有判定閾値: YAML の `occupied_thresh` から自動計算（`--occupied-thresh` で上書き可能）
- `--dry-run` オプションで検出結果・生成座標の確認のみ可能（ファイル書き出しなし）
- CMakeLists.txt の `file(GLOB PYTHON_SCRIPTS "scripts/*.py")` で自動インストール対象になるため追加変更不要

競技運用フロー:
```bash
# [1] マッピング後、PGM 形式でマップ保存（mapping_launch.py 起動中に実行）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "filename: '/home/pi/maps/field'"
ros2 run nav2_map_server map_saver_cli -f /home/pi/maps/field \
  --ros-args -p use_sim_time:=false -p map_subscribe_transient_local:=true

# [2] ウェイポイント自動生成（5秒程度）
python3 src/auto_nav/scripts/generate_waypoints.py \
  --map /home/pi/maps/field.pgm \
  --meta /home/pi/maps/field.yaml \
  --relative src/auto_nav/config/waypoints_relative.yaml \
  --output src/auto_nav/config/waypoints.yaml

# [3] 確認のみ（dry-run）
python3 src/auto_nav/scripts/generate_waypoints.py \
  --map /home/pi/maps/field.pgm \
  --meta /home/pi/maps/field.yaml \
  --relative src/auto_nav/config/waypoints_relative.yaml \
  --output src/auto_nav/config/waypoints.yaml \
  --dry-run
```

---

### 2026-03-02

**フェーズ 9: 競技対応機能（on_arrive シーケンス + start_auto + コート切り替え）** ✅

変更ファイル:
- `src/auto_nav/src/auto_nav/routing_node.py`: on_arrive シーケンス・start_auto/stop_auto・set_court・コート座標変換を追加
- `src/robot_control/src/robot_control/ros2_node.py`: `send_control_command` を変更（hand_control を auto モードでも送信）

**routing_node.py の主な変更点:**

| 追加要素 | 内容 |
|---------|------|
| `_current_court` | "blue"/"red" を保持。`_handle_set_court` で更新 |
| `_pub_rx` | `bluetooth_rx` への Publisher（on_arrive の hand_control コマンド転送用） |
| `_auto_seq_names` | `waypoints.yaml` の `auto_sequence` リストを読み込み |
| `_sequence_abort` | `threading.Event` でシーケンス中断を制御 |
| `_auto_seq_running` / `_auto_seq_index` | `start_auto` シーケンスの進捗管理 |
| `_result_cb` | Nav2 SUCCEEDED 後に `on_arrive` を別スレッドで実行。CANCELED 時は二重発行を防止 |
| `_run_on_arrive_sequence` | `hand_control` アクションを `bluetooth_rx` に publish、`wait` は 50ms チェック付き中断可能スリープ |
| `_on_sequence_done` | シーケンス完了後の処理。auto_sequence 実行中なら次へ進む |
| `_apply_court_transform` | 赤コートは `(x, y, theta) → (x, -y, -theta)` |
| `_handle_start_auto` | `auto_sequence` 先頭から `_advance_auto_sequence` を開始 |
| `_handle_stop_auto` | ゴールキャンセル + シーケンス中断 |
| `_advance_auto_sequence` | 現在インデックスのウェイポイントへ `_handle_goal` を呼び出し |

**状態遷移（変更後）:**
```
MANUAL ──nav_mode:auto──→ AUTO_IDLE ──nav_goal/start_auto──→ NAVIGATING
  ↑                          ↑   ↑                                │
  └──nav_mode:manual──────────┘   └── AUTO_IDLE ←── SEQUENCE ←───┘
                                       ↑
                              全ウェイポイント完了
```

**ros2_node.py の変更点:**
- `send_control_command` を `if self._nav_mode == "auto": return` から改造
- `now / dt / _last_cmd_time` の更新を auto モードでも実行（auto→manual 切り替え時の dt 爆発を防止）
- `wheel_control` の送信は `if self._nav_mode != "auto":` ブロック内に限定
- `hand_control` の送信はブロック外（auto/manual 両モードで毎 50ms 送信）

**on_arrive の hand_control 転送フロー:**
```
routing_node._run_on_arrive_sequence
  → _pub_rx.publish("bluetooth_rx": {"type": "hand_control", ...})
  → robot_control.on_controller_command が受信
  → hands_cntl.set_target(target, control_type, value)
  → send_control_command（auto モードでも動作）が hand_control を hwmc ESP32 に送信
```

**動作確認コマンド:**
```bash
# 起動
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4

# 監視用（別ターミナル）
ros2 topic echo /bluetooth_tx
ros2 topic echo /hand_control

# テスト1: on_arrive シーケンス
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"}' --once
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"nav_goal\", \"waypoint\": \"waypoint_1\"}"}' --once
# 期待: navigating → arrived（on_arrive 完了後）、/hand_control に変化

# テスト2: start_auto シーケンス
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"start_auto\"}"}' --once
# 期待: navigating → arrived(1) → navigating(2) → ... → completed

# テスト3: 中断
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"nav_mode\", \"mode\": \"manual\"}"}' --once
# 期待: cancelled、手動操縦に復帰

# テスト4: コート切り替え（赤）
ros2 topic pub /bluetooth_rx std_msgs/String \
  '{data: "{\"type\": \"set_court\", \"court\": \"red\"}"}' --once
# 期待: Y 座標が反転されてゴール送信
```
