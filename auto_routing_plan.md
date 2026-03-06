# ROS2 + LiDAR 自律走行 実装計画

## 概要

既存のロボット制御システム（4輪メカナムホイール + YDLiDAR）に自律走行機能を追加する。
競技フィールドは毎回 ±5% の寸法誤差があるため、**試合前に毎回マッピング**を行い、
その試合の実際のフィールド形状に合わせた地図を生成してから自律走行する。

---

## 目標アーキテクチャ

```
[目標位置指定]
  └── Bluetooth / WebSocket (JSON: {"type": "nav_goal", "waypoint": "goal_A"})
              ↓
[routing_node]  ←── /map, slam_toolbox の自己位置
  ├── Nav2 ActionClient (NavigateToPose)
  └── /cmd_vel ──→ [cmd_vel_bridge_node] ──→ wheel_control ──→ [ESP32]

[SLAM Toolbox (online_async)] ←── /scan_filtered, /odom ──→ /map, /tf(map→odom)
[odometry_node]               ←── wheel_feedback ──→ /odom_raw
[imu_publisher_node]          ←── wheel_feedback ──→ /imu
[ekf_filter_node]             ←── /odom_raw, /imu ──→ /odom, /tf(odom→base_link)
[YDLiDAR]                     ──→ /scan → [laser_filter] ──→ /scan_filtered
```

---

## 競技運用フロー（フェーズ 8〜10 完成後）

```
① マッピング（試合前）
   mapping_launch.py 起動 → Bluetooth/手動で一周走行
   serialize_map → map_saver_cli（PGM も保存）

② ウェイポイント生成（マッピング直後、5秒程度）
   python3 generate_waypoints.py --map field.pgm --meta field.yaml \
     --relative waypoints_relative.yaml --output waypoints.yaml

③ 本番起動
   auto_nav_launch.py 起動（web_server_node も起動される）

④ コート選択 → Web UI でコート（青/赤）を選択

⑤ 自動制御開始 → Web UI の「自動開始」ボタン → auto_sequence を実行

⑥ フォールバック（自動制御不能時のみ）
   iPad を Bluetooth 接続 → 手動操縦モードに切り替え
```

---

## 実装状況サマリー

| # | 内容 | 状態 | 主要ファイル | 実装日 |
|---|------|------|------------|--------|
| 1 | オドメトリノード | ✅ 完了 | `odometry_node.py` | 2026-02-21 |
| 2 | cmd_vel ブリッジ | ✅ 完了 | `cmd_vel_bridge_node.py` | 2026-02-21 |
| 3 | SLAM セットアップ | ✅ 完了 | `slam_mapping_params.yaml`, `slam_localization_params.yaml` | 2026-02-23 |
| 4 | ウェイポイント管理 | ✅ 完了 | `generate_waypoints.py`, `waypoints_relative.yaml` | 2026-03-01 |
| 5 | 自律走行ノード | ✅ 完了 | `routing_node.py` | 2026-02-27 |
| 6 | Nav2 ナビゲーション | ✅ 完了 | `nav2_params.yaml` | 2026-02-25/27 |
| 7 | ローンチファイル整備 | ✅ 完了 | `mapping_launch.py`, `auto_nav_launch.py` | 2026-02-21/25 |
| 8 | Web コントロール | ✅ 完了 | `web_control/` パッケージ | 2026-03-02 |
| 9 | 競技対応機能 | ✅ 完了 | `routing_node.py` | 2026-03-02 |
| 10 | 壁基準 WP 自動生成 | ✅ 完了 | `generate_waypoints.py` | 2026-03-01 |

---

## フェーズ詳細

### フェーズ 1: オドメトリノード ✅ 完了（2026-02-21）

`wheel_feedback` から 4 輪メカナムの前進運動学で `/odom_raw` を配信する。
`ekf_filter_node` が `/odom_raw` + `/imu` を融合して `/odom` と TF(`odom→base_link`) を配信する（EKF 導入: 2026-02-27）。

**主要ファイル:**
- `src/auto_nav/src/auto_nav/odometry_node.py`（`/odom_raw` 配信）
- `src/auto_nav/src/auto_nav/imu_publisher_node.py`（`/imu` 配信）
- `src/auto_nav/config/ekf_params.yaml`

**注意点:**
- `publish_rate` パラメータではなく `wheel_feedback` 受信レートで駆動（~20Hz）
- 2026-02-26 に odometry_node.py に IMU gz 直接補正を追加したが、2026-02-27 の EKF 移行でその実装は削除済み。現在は `/odom_raw`（エンコーダのみ）+ EKF の構成
- 静止時 IMU gz バイアスドリフト対策: `imu_publisher_node.py` が静止検出（エンコーダ omega < 0.01 rad/s）で gz 共分散を 0.01 → 10.0 に上げ EKF に無視させる（0.23°/秒 → 0.001°/秒）

---

### フェーズ 2: cmd_vel ブリッジノード ✅ 完了（2026-02-21）

`/cmd_vel` (Twist) → 逆運動学 → `wheel_control` (WheelMessage)。`/nav_mode` でモード切り替え。

**主要ファイル:** `src/auto_nav/src/auto_nav/cmd_vel_bridge_node.py`

**注意点:**
- `auto → manual` 切り替え時にゼロ RPM 指令を送信（急停止防止）
- `MIN_RPM = 700.0`: M3508 の静止摩擦を超えられず不動になる問題を解消。700 RPM ≈ 0.19m/s 相当（速すぎる場合は下げる）

---

### フェーズ 3: SLAM セットアップ ✅ 完了（2026-02-23）

**主要ファイル:**
- `src/auto_nav/config/slam_mapping_params.yaml`（online_async マッピング）
- `src/auto_nav/config/slam_localization_params.yaml`（localization モード）

両ファイルとも `scan_topic: /scan_filtered` を使用（タイヤ映り込みフィルタ後）。

**slam_toolbox ライフサイクル管理（重要）:**
`ros2 lifecycle set` はデーモン経由で "Node not found" になるため `ros2 service call` を直接使う。

```bash
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}'
sleep 2
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'
```

`mapping_launch.py` では `TimerAction(period=3.0) + ExecuteProcess` で自動化済み。

**地図保存:**

```bash
# ※ mapping_launch.py 起動中（slam_toolbox が動いている間）に実行すること

# [1] slam_toolbox ネイティブ形式（ローカリゼーション再開用）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "filename: '/home/pi/maps/field'"

# [2] PGM 形式（generate_waypoints.py / map_server 用）
# map_subscribe_transient_local:=true 必須（slam_toolbox は TRANSIENT_LOCAL で /map を配信）
ros2 run nav2_map_server map_saver_cli -f /home/pi/maps/field \
  --ros-args -p use_sim_time:=false -p map_subscribe_transient_local:=true
```

**ローカリゼーション起動:**
```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field  # 拡張子なし
```

**注意点:**
- `LaserScanFootprintFilter` は TF 変換でサイレント失敗するため不使用。`LaserScanRangeFilter` で 0.33m 未満を除去
- `laser_filter_node` に `name=` を設定するとパラメータ読み込み失敗 → `name=` を削除してデフォルト名を使う
- マッピングパラメータ（2026-02-27 調整済み）:

| パラメータ | 値 | 理由 |
|-----------|-----|------|
| `link_match_minimum_response_fine` | 0.4 | 対角壁での重複描画防止 |
| `loop_match_minimum_response_fine` | 0.55 | 低品質ループクロージャを棄却 |
| `loop_match_maximum_variance_coarse` | 1.5 | 偽ループ候補を除外 |

---

### フェーズ 4: ウェイポイント管理 ✅ 完了（2026-03-01）

**主方針:** `generate_waypoints.py` でウェイポイントを自動生成する。
`waypoints_relative.yaml` にフィールド壁からの相対距離でウェイポイントを事前定義し、マッピング後に自動変換する。

**主要ファイル:**
- `src/auto_nav/config/waypoints_relative.yaml`（相対座標定義）
- `src/auto_nav/scripts/generate_waypoints.py`（変換スクリプト）
- `src/auto_nav/config/waypoints.yaml`（生成された絶対座標、要 PGM マップ）

**運用手順（マッピング後）:**
```bash
python3 src/auto_nav/scripts/generate_waypoints.py \
  --map /home/pi/maps/field.pgm \
  --meta /home/pi/maps/field.yaml \
  --relative src/auto_nav/config/waypoints_relative.yaml \
  --output src/auto_nav/config/waypoints.yaml

# 確認のみ（ファイル書き出しなし）
python3 ... --dry-run
```

詳細仕様（フォーマット・壁検出アルゴリズム）はフェーズ 10 を参照。

---

### フェーズ 5: 自律走行ノード ✅ 完了（2026-02-27）

Bluetooth / WebSocket コマンドを受け取り、Nav2 ActionClient でゴールを送信する制御ノード。

**主要ファイル:**
- `src/auto_nav/src/auto_nav/routing_node.py`
- `src/auto_nav/scripts/launch_routing.py`

**状態遷移:**
```
MANUAL ──nav_mode:auto──→ AUTO_IDLE ──nav_goal/start_auto──→ NAVIGATING
  ↑                          ↑   ↑                                │
  └──nav_mode:manual──────────┘   └── AUTO_IDLE ←── SEQUENCE ←───┘
                                       ↑
                              全ウェイポイント完了
```

**動作確認コマンド:**
```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4
ros2 topic echo /bluetooth_tx
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"' --once
ros2 topic pub /bluetooth_rx std_msgs/msg/String \
  'data: "{\"type\": \"nav_goal\", \"waypoint\": \"waypoint_1\"}"' --once
```

**重要: ActionClient + SingleThreadedExecutor 問題:**
`ActionClient` 初期化時に Nav2 の `_action/status` トピックを受信する Waitable が登録され、`SingleThreadedExecutor` では bluetooth_rx コールバックが一切実行されなくなる。`MultiThreadedExecutor` を使うこと:
```python
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

---

### フェーズ 6: Nav2 ナビゲーション ✅ 完了（2026-02-25/27）

**主要ファイル:** `src/auto_nav/config/nav2_params.yaml`

設定概要:
- プランナー: NavfnPlanner（A*: `use_astar: true`, tolerance: 0.5m）
- コントローラー: DWBLocalPlanner（メカナム対応: `max_vel_y: 0.5`, `vy_samples: 10`）
- フットプリント: `[[0.22, 0.29], [-0.22, 0.29], [-0.22, -0.29], [0.22, -0.29]]`
- コストマップ: `scan_topic: /scan_filtered`, `inflation_radius: 0.55m`

**DWB チューニング結果（2026-02-27）:**

| パラメータ | Before | After | 理由 |
|-----------|--------|-------|------|
| `general_goal_checker.xy_goal_tolerance` | 0.25 | 0.08 | ±8cm 精度目標 |
| `general_goal_checker.yaw_goal_tolerance` | 0.25 | 0.10 | ±6° 精度目標 |
| `FollowPath.max_vel_x` | 0.5 | 1.0 | 速度倍増 |
| `FollowPath.max_speed_xy` | 0.5 | 1.0 | 速度倍増 |
| `FollowPath.sim_time` | 1.7 | 1.2 | ふらつき抑制 |
| `FollowPath.xy_goal_tolerance` | 0.25 | 0.08 | RotateToGoal トリガー精度 |
| `FollowPath.PathAlign.scale` | 32.0 | 8.0 | ふらつきの主原因を除去 |
| `velocity_smoother.max_velocity` | [0.5, 0.5, 1.0] | [1.0, 0.5, 1.0] | DWB 上限に合わせる |

**Nav2 Jazzy 固有の注意点:**
- `navigation_launch.py` が管理するノードは 10 個（`docking_server` など旧版にないノードを含む）。全ノードのパラメータを `nav2_params.yaml` に定義すること
- `bt_navigator` の `plugin_lib_names` を明示するとビルトイン BT ノード ID が二重登録されて FATAL → 削除してデフォルト動作を使う
- `collision_monitor`: `observation_sources` 未定義で configure 失敗 → params 追加で解決
- `docking_server`: `dock_plugins` 未定義で configure 失敗 → params 追加で解決

---

### フェーズ 7: ローンチファイル整備 ✅ 完了

**主要ファイル:**
- `src/auto_nav/launch/mapping_launch.py`
- `src/auto_nav/launch/auto_nav_launch.py`

**mapping_launch.py 起動ノード:**
micro_ros_agent / ydlidar_ros2_driver / laser_filter_node / static_tf（base_link→laser_frame）
/ odometry_node / imu_publisher_node / ekf_node / cmd_vel_bridge_node
/ slam_toolbox / robot_control / bt_communication

**起動コマンド:**
```bash
ros2 launch auto_nav mapping_launch.py                      # シリアル（デフォルト）
ros2 launch auto_nav mapping_launch.py serial_port:=/dev/ttyUSB1  # USB1 の場合
ros2 launch auto_nav mapping_launch.py transport:=udp4      # WiFi UDP
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4
```

**注意:** `base_link → laser_frame` の static_tf は名前付き引数で設定。
LiDAR 取付向き: `--yaw 1.5708`（0度方向がロボット左向き）、`--pitch 3.14159`（上下反転補正）。z=0.15m は仮設定のため実機実測で調整すること。

---

### フェーズ 8: Web コントロールシステム（未実装）

**背景:** 競技規則により試合中は一切の外部通信が禁止。Bluetooth コントローラは自動制御失敗時の手動フォールバック専用とし、通常運用は WebSocket 経由に移行する。

**基本方針:** UI は既存の `khr2026_team1_bt_controller`（React + TypeScript + Vite）に **WebSocket 接続ページを追加**する。

```
[khr2026_team1_bt_controller (ビルド済み SPA)]
        ↕ WebSocket (ws://<raspi-ip>:8080/ws)
[web_server_node（本リポジトリ: web_control パッケージ）]
        ↕ bluetooth_rx / bluetooth_tx (std_msgs/String)
[routing_node / robot_control]
```

オペレーターのスマホ/タブレットはラズパイの Wi-Fi AP に接続し、`http://<raspi-ip>:5173` または GitHub Pages の URL を開く。ラズパイ側は WebSocket サーバーのみ起動すればよい。

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

**パッケージ作成手順（実装時）:**
```bash
cd src && ros2 pkg create web_control
cd web_control
uv init . --lib --python-preference only-system
uv venv --system-site-packages
uv add aiohttp
```

**web_server_node.py の動作仕様:**
1. `aiohttp` で WebSocket サーバーをポート 8080 で起動（`ws://0.0.0.0:8080/ws`）
2. 接続中の全クライアントをセットで管理し、ブロードキャスト
3. `Access-Control-Allow-Origin: *` ヘッダーを HTTP Upgrade レスポンスに付与（GitHub Pages 等からの接続を許可）

**asyncio + rclpy 統合方針:**
- rclpy `MultiThreadedExecutor` でスピン（メインスレッド）
- `aiohttp` の asyncio ループは `threading.Thread` + `asyncio.new_event_loop()` で別スレッドで起動
- ROS2 コールバック → WebSocket: `asyncio.run_coroutine_threadsafe(broadcast(msg), loop)`
- WebSocket → ROS2: `publisher.publish()`（rclpy の publish はスレッドセーフ）

**ROS2 インターフェース:**
- Sub: `bluetooth_tx` (String) → WebSocket 全クライアントにブロードキャスト
- Pub: `bluetooth_rx` (String) ← WebSocket から受信したコマンドを転送

---

#### 8-2: `khr2026_team1_bt_controller` リポジトリへの追加

既存スタック（React 19 + TypeScript + Vite + Tailwind CSS 4 + React Router 7）をそのまま使用。

**`src/hooks/useWebSocketConnect.tsx`（新規）** — `useBluetoothConnect.tsx` と同等のインターフェース:

```typescript
interface UseWebSocketConnectReturn {
  isConnected: boolean
  connect: (url: string) => Promise<void>   // 例: "ws://192.168.4.1:8080/ws"
  disconnect: () => void
  sendJson: (data: unknown) => void
  lastMessage: unknown | null
}
```

**`src/routes/Index.tsx`（変更）** — 以下のメニューを追加:

| メニュー | 説明 |
|---------|------|
| Controller（既存） | Bluetooth で手動コントロール |
| PID Tuning（既存） | 足回り PID ゲイン調整 |
| **Auto Nav（新規）** | **自律走行の制御・モニタリング（WebSocket）** |
| **Controller (WS)（新規）** | **WebSocket 経由で手動コントロール（競技本番用）** |

**`src/routes/Controller.tsx`（変更）** — WebSocket 接続オプションを追加（`?mode=ws&url=ws://...` で接続手段を切り替え）。

**新規ルート: `src/routes/AutoNav/`**

```
src/routes/AutoNav/
├── index.tsx           ← AutoNav ページ本体
├── useAutoNav.ts       ← WebSocket 接続 + 状態管理フック
└── CourtSelector.tsx   ← コート選択コンポーネント
```

**`AutoNav/index.tsx` の UI 仕様:**
- **接続セクション:** WebSocket URL 入力欄（デフォルト: `ws://192.168.4.1:8080/ws`）、接続/切断ボタン
- **コート選択:** 「青コート」「赤コート」ボタン → `{"type": "set_court", "court": "blue"|"red"}` を送信
- **制御:** 「自動開始」`{"type": "start_auto"}`、「停止」`{"type": "stop_auto"}`（誤タップ防止ダイアログ付き）
- **ステータス:** モード表示、現在ウェイポイント名＋進捗（例: `1 / 3`）、フィールド SVG でロボット位置表示
- **ログ:** `bluetooth_tx` から受信した `nav_status` メッセージを時系列表示（最新 20 件）

---

#### 8-3: WebSocket メッセージプロトコル

Bluetooth 版と完全に同一フォーマット。「共通リファレンス: Bluetooth/WebSocket コマンド仕様」を参照。
`web_server_node` は `bluetooth_tx` の内容をそのままブラウザにブロードキャストするため、PID チューニング画面も WebSocket 接続で動作する。

---

### フェーズ 9: 競技対応機能 ✅ 完了（2026-03-02）

**変更ファイル:**
- `src/auto_nav/src/auto_nav/routing_node.py`（on_arrive、start_auto/stop_auto、set_court、コート座標変換を追加）
- `src/robot_control/src/robot_control/ros2_node.py`（hand_control を auto モードでも送信するよう変更）

**追加機能:**

| 機能 | 実装 |
|------|------|
| コート切り替え | `set_court` コマンドで `_current_court` を更新。赤コートは `(x, y, θ) → (-x, y, π-θ)` |
| on_arrive シーケンス | Nav2 SUCCEEDED 後、on_arrive リストを別スレッドで順次実行 |
| start_auto | `auto_sequence` の先頭から `_advance_auto_sequence` を開始 |
| stop_auto | ゴールキャンセル + `_sequence_abort` Event でシーケンス中断 |

**on_arrive の hand_control 転送フロー:**
```
routing_node._run_on_arrive_sequence
  → _pub_rx.publish(bluetooth_rx: {"type": "hand_control", ...})
  → robot_control.on_controller_command が受信
  → send_control_command（auto モードでも hand_control は動作）が hwmc ESP32 に送信
```

**ros2_node.py の変更:** `wheel_control` の送信は `if self._nav_mode != "auto":` ブロック内に限定。`hand_control` の送信はブロック外（auto/manual 両モードで毎 50ms 送信）。

**動作確認コマンド:**
```bash
ros2 launch auto_nav auto_nav_launch.py map:=/home/pi/maps/field transport:=udp4
ros2 topic echo /bluetooth_tx
ros2 topic echo /hand_control

# on_arrive テスト
ros2 topic pub /bluetooth_rx std_msgs/String '{data: "{\"type\": \"nav_mode\", \"mode\": \"auto\"}"}' --once
ros2 topic pub /bluetooth_rx std_msgs/String '{data: "{\"type\": \"nav_goal\", \"waypoint\": \"waypoint_1\"}"}' --once

# start_auto テスト（全シーケンス）
ros2 topic pub /bluetooth_rx std_msgs/String '{data: "{\"type\": \"start_auto\"}"}' --once

# コート切り替えテスト
ros2 topic pub /bluetooth_rx std_msgs/String '{data: "{\"type\": \"set_court\", \"court\": \"red\"}"}' --once
```

---

### フェーズ 11: エラーアラートとシーケンス途中再開 ✅ 完了（2026-03-04）

#### 「続行不可能」の定義

`nav_status: "error"` 受信時（Nav2 が `STATUS_SUCCEEDED` / `STATUS_CANCELED` 以外を返した場合）。auto sequence 中・単発 nav_goal どちらの失敗でも点滅する。

#### rspi 側 `routing_node.py`

- `start_auto` コマンドに `from_index` フィールドを追加（省略時 = 0）
- `from_index > 0` の場合: `_relocate_and_start()` を別スレッドで起動
  1. 指定ウェイポイント座標を `/initialpose` に publish（slam_toolbox の再ローカライズ起点を更新）
  2. 3 秒カウントダウン（1 秒ごとに `nav_status: "relocating"` を送信）
  3. カウントダウン完了後にナビゲーション開始
- `navigating` / `arrived` / `error` メッセージに `seq_index` / `seq_total` を追加
- auto sequence 中の `arrived`（on_arrive 完了後）も `bluetooth_tx` に送信するよう変更

#### bt_controller 側

- `NavStatus` に `'RELOCATING'` を追加
- `isAlertFlashing`: error 受信で true → resume/stop/manual で false
- `failedSeqIndex`: エラーが発生したシーケンスインデックスを保持
- `relocatingCountdown`: カウントダウン残り秒数
- `sendStartAutoFrom(fromIndex)`: `{"type": "start_auto", "from_index": N}` を送信
- 点滅オーバーレイ: `fixed inset-0 z-40 pointer-events-none`、赤 ↔ 白を 500ms 間隔で切り替え
- シーケンスリスト: 各ウェイポイント行に「ここから」ボタン。失敗行はオレンジ強調・完了行は取り消し線
- `from_index > 0` 時: 「ロボットを waypoint_N の位置に置きましたか？」確認ダイアログを表示

#### 競技時のリトライフロー

```
① エラー発生 → 画面が赤白点滅（point-events-none なのでボタン操作は継続可）
② オペレーター: 「停止」ボタン → 手動でロボットを対象ウェイポイント位置に移動
③ シーケンスリストで失敗行（オレンジ）の「ここから」ボタンを押す
④ 確認ダイアログ → OK
⑤ rspi が /initialpose を publish → 3 秒カウントダウン（UI に残り秒表示）
⑥ ナビゲーション再開（点滅も停止）
```

---

### フェーズ 10: 壁基準ウェイポイント生成 ✅ 完了（2026-03-01）

マッピング後にスクリプトで `waypoints_relative.yaml`（壁からの相対距離定義）を
`waypoints.yaml`（絶対座標）に自動変換する。±5% の寸法誤差は PGM マップに自動反映される。

**主要ファイル:**
- `src/auto_nav/config/waypoints_relative.yaml`
- `src/auto_nav/scripts/generate_waypoints.py`

**処理フロー:**
1. PIL で PGM を読み込み（P2/P5 両対応、Pillow 不要。純粋 Python で実装）
2. `field.yaml` から `resolution`、`origin` を取得
3. 占有閾値（`field.yaml` の `occupied_thresh` から自動計算）以上のピクセルを壁と判定。各行/列の `statistics.median` でノイズ除去して壁外縁座標を検出
4. 相対定義を絶対座標に変換（例: `from_wall=south, distance=0.5` → `y = wall_south_y + 0.5`）
5. `waypoints.yaml` に書き出し（routing_node がそのまま読める）

**壁検出の前提:** ロボットがフィールド全周を走行して壁が途切れていないことが必要。`--dry-run` で検出結果を目視確認すること。

---

## 共通リファレンス

### Bluetooth / WebSocket コマンド仕様

**ロボットへの送信:**

| type | フィールド | 説明 |
|------|-----------|------|
| `nav_mode` | `mode: "auto"\|"manual"` | ナビゲーションモード切り替え |
| `nav_goal` | `waypoint: string` | ウェイポイント名でゴール指定 |
| `nav_goal` | `x, y, theta: float` | 直接座標でゴール指定 |
| `set_court` | `court: "blue"\|"red"` | コート選択（赤コートは Y 軸反転） |
| `start_auto` | — | `auto_sequence` を先頭から実行 |
| `stop_auto` | — | 自動制御停止（シーケンス中断） |
| `joystick` | `l_x, l_y, r: int (-10〜10)` | 手動走行 |
| `hand_control` | `target, control_type, action` | ハンド操作 |
| `pid_gains` | `kp, ki, kd: float` | PID ゲイン調整 |

**ロボットからの受信（`bluetooth_tx`）:**

| 発生元 | type / フィールド | 説明 |
|--------|-------------|------|
| routing_node | `nav_status: "mode"`, `mode` | モード変更通知 |
| routing_node | `nav_status: "navigating"`, `waypoint`, `progress` | 走行中（例: `"1/3"`） |
| routing_node | `nav_status: "arrived"`, `waypoint`, `progress` | 到達通知（on_arrive 完了後） |
| routing_node | `nav_status: "completed"` | 全 auto_sequence 完了 |
| routing_node | `nav_status: "cancelled"` | キャンセル通知 |
| routing_node | `nav_status: "error"`, `message` | エラー通知 |
| robot_control | `m3508_rpms`, `yagura`, `ring`, ... | フィードバック |

---

### waypoints.yaml フォーマット（on_arrive 対応版）

```yaml
auto_sequence:
  - waypoint_1
  - waypoint_2

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
        duration: 1.5
      - action: hand_control
        target: ring_1
        control_type: state
        action_value: open
  waypoint_2:
    x: -1.859
    y: 2.419
    theta: 0.0
    on_arrive: []
```

**`on_arrive` で使えるアクション種別:**

| action | 必須フィールド | 説明 |
|--------|-------------|------|
| `hand_control` | `target, control_type, action_value` | `bluetooth_rx` に手動制御コマンドを発行 |
| `wait` | `duration` (秒) | 指定秒数待機（50ms チェック付き中断可能スリープ） |

**`target`:** `yagura_1`, `yagura_2`, `ring_1`, `ring_2`

| control_type | action_value の値 |
|-------------|---------------|
| `pos`（yagura） | `up`, `down`, `stopped` |
| `state`（yagura） | `open`, `close`, `stopped` |
| `pos`（ring） | `pickup`, `yagura`, `honmaru`, `stopped` |
| `state`（ring） | `open`, `close`, `stopped` |

---

### waypoints_relative.yaml フォーマット

```yaml
# 方角の定義:
#   north: マッピング開始時にロボットが向いていた方向の壁
#   south: その反対の壁、east: ロボットの右側、west: ロボットの左側
auto_sequence:
  - waypoint_1
  - waypoint_2

waypoints:
  waypoint_1:
    from_wall: south
    distance: 0.5         # 壁面（内側面）からの距離 [m]
    cross_wall: west
    cross_distance: 1.0
    theta: 1.57
    on_arrive:
      - action: hand_control
        target: ring_1
        control_type: state
        action_value: open
```

**コート対称軸の前提:** 赤コートの Y 軸反転はマッピング開始時のロボット向きを X 軸としている。マッピング時は必ずフィールド中央線に正対して配置すること。

---

### TF ツリー構成

```
map
 └── odom                      ← slam_toolbox が配信（~9.5Hz）
      └── base_link            ← ekf_filter_node が配信（robot_localization、~17Hz）
           └── laser_frame     ← static_tf（yaw=1.5708, pitch=3.14159, z=0.15m）
```

`odometry_node` は `/odom_raw` のみ配信。`odom→base_link` TF は EKF が担当。

---

### リスクと注意点

1. **オドメトリ精度**: メカナムホイールはスリップしやすい。マッピング中は 0.2m/s 以下で走行すること。
2. **手動/自動モード競合**: manual 時は robot_control が、auto 時は cmd_vel_bridge が wheel_control をパブリッシュ。両方が同時にパブリッシュしないようモード切り替えロジック実装済み。
3. **LiDAR 取付位置の TF**: `laser_frame` のズレはコストマップ精度に直結する。z=0.15m は仮設定のため実機で計測して確認すること。
4. **YDLiDAR のデッドゾーン**: 0.1m 以内はデッドゾーン。フットプリントにマージンを持たせること。
5. **ウェイポイント管理**: `generate_waypoints.py` で自動生成。手動調整が必要な場合は `waypoints.yaml` を直接編集する。
6. **Nav2 のメカナムホイール対応**: DWB で Y 方向速度を有効化済み（`max_vel_y`, `vy_samples`）。
7. **LiDAR タイヤ映り込み**: `LaserScanRangeFilter` で 0.33m 未満を除去（`/scan_filtered`）。
8. **コート対称軸の前提**: 赤コートの Y 軸反転はマッピング開始時のロボット向きを X 軸としている。マッピング時は必ずフィールド中央線に正対して配置すること。
9. **asyncio + rclpy の統合**: web_server_node は aiohttp の asyncio ループと rclpy のループを別スレッドで動かす。`asyncio.run_coroutine_threadsafe` / スレッドセーフ publish 以外での跨ぎ操作は禁止。
10. **on_arrive 実行中のブロッキング**: `time.sleep` を rclpy メインスレッドで呼ぶとコールバックが止まる。on_arrive シーケンスは専用スレッドで実行すること。
11. **壁検出の前提**: `generate_waypoints.py` はロボットがフィールド全周を走行して壁が途切れていないことを前提とする。

---

### 必要パッケージ

```bash
sudo apt install \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-dwb-core \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-nav2-map-server \
  ros-jazzy-laser-filters \
  ros-jazzy-robot-localization
```

---

## 作業記録

### 2026-02-21
- `auto_nav` パッケージ新規作成（フェーズ 1, 2）
- `odometry_node.py`: wheel_feedback → /odom + TF(odom→base_link)
- `cmd_vel_bridge_node.py`: /cmd_vel → wheel_control、/nav_mode でモード切り替え
- `mapping_launch.py` 作成（serial_port 引数でポート変更可能）

### 2026-02-22
- /odom 配信 20Hz、モード切り替え正常確認
- scripts/*.py に実行権限がなかった → `chmod +x` で修正（以後スクリプト追加時は要注意）
- `LaserScanRangeFilter` で /scan_filtered 実装（LaserScanFootprintFilter は TF 変換でサイレント失敗するため不使用）
- slam_toolbox ライフサイクル問題解決: `ros2 service call /slam_toolbox/change_state` を直接呼び出し（デーモン経由不可）

### 2026-02-23
- コードレビュー修正（slam params の enable_interactive_mode/do_loop_closing、odometry_node の theta 積分順序修正・共分散行列設定）
- 実機マッピング成功: 1149×540 cells、field.posegraph (19MB) + field.data (8.9MB) 生成

### 2026-02-25
- `nav2_params.yaml` 新規作成（フェーズ 6）
- `auto_nav_launch.py` に Nav2 組み込み（navigation_launch.py 経由）
- Nav2 Jazzy 固有のトラブル解決: plugin_lib_names の二重登録、docking_server/collision_monitor のパラメータ追加

### 2026-02-26
- laser_frame TF の yaw/pitch 設定バグ修正（位置引数 → 名前付き引数）
- **IMU gz 直接補正を odometry_node.py に追加（暫定）** → 翌日 EKF 移行により当該実装を削除

### 2026-02-27
- **EKF センサフュージョン導入（フェーズ 1 更新）**: odometry_node.py から IMU 融合・TF 配信を除去し `/odom_raw` のみに変更。imu_publisher_node.py（新規）+ ekf_params.yaml（新規）を追加。静止時 IMU gz バイアス対策として静止検出で gz 共分散を上げる実装を追加（0.23°/秒 → 0.001°/秒）
- SLAM マッピングパラメータ調整（対角壁重複描画・二重壁解消）
- **routing_node.py 実装（フェーズ 5）**: ActionClient + SingleThreadedExecutor で bluetooth_rx コールバックが無音になる問題 → MultiThreadedExecutor に変更で解決
- DWB チューニング + MIN_RPM=700 追加（フェーズ 6 詳細参照）
- フェーズ 8〜10 の要件・実装方針を策定

### 2026-03-01
- **フェーズ 10**: `generate_waypoints.py` + `waypoints_relative.yaml` 実装（PGM 純粋 Python 読み込み、P2/P5 両対応、`--dry-run` オプション付き）

### 2026-03-03
- **手動制御コート選択機能追加**: `Controller.tsx` にコート選択ボタン（青/赤）を追加
  - 青コート選択時: l_x・l_y を符号反転（ロボットが南向きのため、北を前にするための 180° 回転）
  - 赤コート選択時: 変換なし（ロボットが北向きのため、そのまま）
  - コート選択時に `set_court` コマンドを送信（manual→auto 切替時に routing_node と同期）
  - UI: BT 接続ボタン横に「青」「赤」ボタンを追加

- **バグ修正**: `routing_node.py` の `_apply_court_transform` の赤コート変換が誤り
  - 誤: `(x, -y, -θ)` ← 東西線（横線）対称になっていた
  - 正: `(-x, y, π-θ)` ← 南北線（縦線）対称
  - フィールドは赤（左）・青（右）コートが横並び、北南線で線対称。X軸反転が正しい
  - θ 変換: 縦線反転では `π - θ`（東向き→西向き、北向き→北向きのまま）

### 2026-03-05

- **直進蛇行・角度ずれ修正**: HeadingPID の kp が度単位 error に rad/s スケールで掛かる問題を修正
  - 根本原因: `heading_pid.py` は error を度単位で計算するが、output を rad/s として IK に渡すため
    kp=1.0 は実効的に 57 倍の大きさで動作し、1° ドリフト → ~1548 RPM の過剰補正
  - **Fix 1** (`cmd_vel_bridge_node.py` L45): `kp=1.0, kd=0.05` → `kp=0.02, kd=0.001`
    1° error → 0.02 rad/s → ~31 RPM（全速の 1.5%）と適正化
  - **Fix 2** (`cmd_vel_bridge_node.py` L140): `OMEGA_THRESHOLD=0.05` → `0.02` rad/s
    Nav2 のゆっくりしたカーブ中も PID を無効化しやすくする
  - **Fix 3** (`imu_publisher_node.py`): gz 符号診断ログを追加
    回転中 (|omega_enc| > 0.05) に `ratio=gz/omega_enc` を DEBUG 出力
    ratio≈+1.0 → 正常、ratio≈-1.0 → gz 反転が必要

  確認コマンド:
  ```bash
  # yaw 安定確認（直進中に値が跳ねないこと）
  ros2 topic echo /odom --field pose.pose.orientation

  # gz 符号診断（旋回しながら確認）
  ros2 run auto_nav imu_publisher_node --ros-args --log-level debug
  ```

### 2026-03-04

- **フェーズ 11**: エラーアラートとシーケンス途中再開を実装
  - `routing_node.py`: `start_auto` に `from_index` オプション追加、`_relocate_and_start()` スレッドで initialpose publish + 3 秒カウントダウン後にナビ開始、navigating/arrived/error メッセージに `seq_index`/`seq_total` 追加、auto seq 中の `arrived` メッセージを送信するよう変更
  - `useAutoNav.ts`: `isAlertFlashing`/`failedSeqIndex`/`relocatingCountdown` 状態追加、`sendStartAutoFrom(fromIndex)` 追加、`RELOCATING` ステータス追加
  - `AutoNav/index.tsx`: 赤白交互点滅オーバーレイ（error 時）、シーケンスリストに「ここから」ボタン、リローカライズカウントダウン表示、from_index > 0 の確認ダイアログ追加

---

### 2026-03-02
- **フェーズ 9**: routing_node.py に on_arrive シーケンス・start_auto/stop_auto・set_court・コート座標変換を追加
- ros2_node.py: hand_control は auto/manual 両モードで送信するよう変更（wheel_control のみ auto モード時にスキップ）

- **フェーズ 8**: WebSocket コントロールシステム実装
  - `src/web_control/` パッケージ新規作成（aiohttp WebSocket ブリッジ、port 8080）
  - `auto_nav_launch.py` に `web_server_node` を追加（`_WEB_PYTHONPATH` で aiohttp venv を参照）
  - bt_controller: `useWebSocketConnect.tsx`（WS 接続管理フック）
  - bt_controller: `CourtSelector.tsx`（コート選択コンポーネント）
  - bt_controller: `useAutoNav.ts` をリファクタリング（引数を `sendJson` に抽象化、`court`/`progress`/`sendSetCourt`/`sendStartAuto`/`sendStopAuto` を追加）
  - bt_controller: `AutoNav/index.tsx` を更新（BLE/WS タブ切り替え、コート選択、自動シーケンス制御 UI）

  セットアップ（raspi 上で実行）:
  ```bash
  cd /home/pi/DRC/khr2026_team1_rspi/src/web_control
  uv venv --system-site-packages
  uv add aiohttp
  cd /home/pi/DRC/khr2026_team1_rspi
  colcon build --packages-select web_control --symlink-install
  source install/setup.bash
  ```
