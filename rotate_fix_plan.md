# 点群回転問題の修正プラン

## Context

ロボットが旋回すると、RViz 上で点群（LiDARスキャンデータ）が壁に固定されず、ロボットと一緒に回転してしまう。
これは「ロボットの回転量」をシステムが正しく認識できていないことを意味し、TF ツリー（odom → base_link → laser_frame）のいずれかの変換に不整合がある。

根本原因の候補は大きく分けて **7 つ** あり、複数が同時に存在する可能性がある。
以下に優先度順で整理し、それぞれのデバッグ手順と修正方法を記載する。

---

## 原因 1: IMU gz（ヨー角速度）の符号が実機と逆（最有力）

### 該当ファイル
- `khr2026_team1_cwmc/lib/lsm9ds1_control/src/lsm9ds1_control.cpp:76-78`
- `khr2026_team1_rspi/src/auto_nav/src/auto_nav/imu_publisher_node.py:69,105`

### 問題の詳細
LSM9DS1 の磁気センサーには搭載向き補正（`mx = -imu.my`, `my = -imu.mx`）が適用されているが、
**ジャイロスコープ（gz）には一切の軸変換・符号補正が行われていない**。

```cpp
// lsm9ds1_control.cpp - 磁気は補正あり、ジャイロは補正なし
v.gx = imu.calcGyro(imu.gx);  // 補正なし
v.gy = imu.calcGyro(imu.gy);  // 補正なし
v.gz = imu.calcGyro(imu.gz);  // 補正なし ← ここが怪しい
```

IMU の物理的な取付向きによっては、ROS 標準（反時計回り = 正）と逆の符号で出力されている可能性がある。
EKF が odom_raw（エンコーダ）と imu（ジャイロ）の両方から vyaw を受け取っているため、
符号が逆だと推定ヨー角が破綻し、点群が回転する。

### デバッグ手順
1. ロボットを手で **左回り（反時計回り）** にゆっくり回す
2. `ros2 topic echo /imu --field angular_velocity.z` → **正の値** になるべき
3. `ros2 topic echo /odom_raw --field twist.twist.angular.z` → **正の値** になるべき
4. 既存の診断ログも活用:
   ```bash
   ros2 run auto_nav launch_imu_publisher.py --ros-args --log-level debug
   ```
   → `ratio` が `+1.0` 付近なら正常、`-1.0` 付近なら gz 反転が必要

### 修正方法
**A) Raspberry Pi 側で反転**（推奨 — ESP32 の再フラッシュ不要）:
```python
# imu_publisher_node.py:69
gz_raw = -(imu_val.gz * math.pi / 180.0)  # 符号反転
```

**B) ESP32 ファームウェア側で反転**:
```cpp
// lsm9ds1_control.cpp:78
v.gz = -imu.calcGyro(imu.gz);  // 搭載向きに合わせて反転
```

---

## 原因 2: LiDAR の static TF が「裏返し」（pitch=π）

### 該当ファイル
- `khr2026_team1_rspi/src/auto_nav/launch/mapping_launch.py:191-196`
- `khr2026_team1_rspi/src/auto_nav/launch/auto_nav_launch.py:145-149`

### 問題の詳細
```python
"--yaw", "1.5708", "--pitch", "3.14159", "--roll", "0.0",
```
`pitch = 3.14159`（180度）は「LiDAR が上下逆さまに取り付けられている」ことを意味する。
**実機で LiDAR が普通に上向きに取り付けられている場合**、スキャンデータが鏡像反転し、
ロボットが左に回ると点群が右に回る（2倍速で回転しているように見える）現象が起きる。

### デバッグ手順
1. RViz で Fixed Frame を `base_link` に設定
2. `/scan` を表示
3. ロボットの前方に物体を置く → RViz 上でもロボット前方に点群が表示されるか確認
4. ロボットの右側に物体を置く → RViz 上でも右側に表示されるか確認
5. 鏡像になっている場合は pitch が原因

### 修正方法
LiDAR が上向き設置なら:
```python
"--yaw", "1.5708", "--pitch", "0.0", "--roll", "0.0",
```

LiDAR が下向き設置（裏返し）なら現状の `pitch=3.14159` で正しい。

**注意**: yaw=1.5708（90度）は LiDAR の前方方向とロボットの前方方向のずれを補正するもので、
これは実機の取り付け角に依存する。

---

## 原因 3: エンコーダ omega の符号が逆

### 該当ファイル
- `khr2026_team1_rspi/src/auto_nav/src/auto_nav/odometry_node.py:78`

### 問題の詳細
```python
omega = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * G)
```
この式は逆運動学（cmd_vel_bridge_node.py の式）と数学的に整合しているが、
**M3508 モータの RPM フィードバックの符号規約が想定と異なる場合**、omega の符号が逆になる。

具体的には、CAN ID 0x201-0x204 から返される RPM 値の正負が、
コマンド送信時の RPM 正負と一致しているかどうかに依存する。

### デバッグ手順
1. ロボットを手で **左回り（反時計回り）** に回す
2. `ros2 topic echo /odom_raw --field twist.twist.angular.z`
3. **正の値** が出ていれば OK、**負の値** なら符号を反転する

### 修正方法
```python
# odometry_node.py:78
omega = (v_fl + v_fr + v_rl + v_rr) / (4.0 * G)  # マイナスを外す
```

---

## 原因 4: IMU とエンコーダの符号が逆（EKF 内で競合）

### 該当ファイル
- `khr2026_team1_rspi/src/auto_nav/config/ekf_params.yaml:17-18,29`

### 問題の詳細
EKF の設定で、odom0（エンコーダ）と imu0（IMU）の **両方** から vyaw を受け取っている:
```yaml
odom0_config: [..., true]    # vyaw = true（12番目）
imu0_config:  [..., true]    # vyaw = true（12番目）
```

両者の符号が一致していれば相互補完するが、**符号が逆だと EKF の推定ヨー角が振動・破綻する**。
これが原因 1 と原因 3 の組み合わせで発生する。

### デバッグ手順
1. 上記の原因 1・3 のデバッグで、IMU とエンコーダの両方の符号を確認
2. 両方とも反時計回り = 正 であることを確認

### 修正方法
原因 1 と原因 3 を修正すれば自動的に解決する。
応急処置として、一方のソースの vyaw を false にすることで切り分けも可能:
```yaml
# imu0_config の vyaw を false にしてエンコーダのみで動作確認
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, false,    # vyaw = false
              false, false, false]
```

---

## 原因 5: IMU ジャイロの軸マッピングが未補正

### 該当ファイル
- `khr2026_team1_cwmc/lib/lsm9ds1_control/src/lsm9ds1_control.cpp:76-78`

### 問題の詳細
磁気センサーでは物理的な搭載向きに合わせて軸の入れ替え・符号反転を行っている:
```cpp
float mx = -imu.calcMag(imu.my);  // Y → X（符号反転）
float my = -imu.calcMag(imu.mx);  // X → Y（符号反転）
```

しかしジャイロでは同じ変換が適用されていない:
```cpp
v.gx = imu.calcGyro(imu.gx);  // そのまま
v.gy = imu.calcGyro(imu.gy);  // そのまま
v.gz = imu.calcGyro(imu.gz);  // そのまま
```

IMU が基板上で回転して搭載されている場合、磁気と同様にジャイロにも軸変換が必要。
ただし、**gz（ヨー角速度）のみ EKF で使用している**ため、影響は gz の符号のみ。

磁気の変換（X↔Y入れ替え + 符号反転）を考えると、IMU は Z 軸周りに 90度 + 鏡像で搭載されている可能性がある。
この場合 gz の符号も反転が必要になる（原因 1 と同じ結論）。

### 修正方法
原因 1 の修正と統合する。

---

## 原因 6: タイムスタンプの同期ズレ

### 該当ファイル
- `khr2026_team1_rspi/src/auto_nav/src/auto_nav/imu_publisher_node.py:96`
- `khr2026_team1_rspi/src/auto_nav/src/auto_nav/odometry_node.py:91`
- `khr2026_team1_rspi/src/auto_nav/src/auto_nav/scan_relay_node.py:14`
- `khr2026_team1_cwmc/src/main.cpp:345`（50ms タイマー）

### 問題の詳細
- LiDAR スキャンは YDLiDAR ドライバが独自のタイムスタンプを付与
- IMU/エンコーダは Raspberry Pi 側の `rclcpp::Clock` でタイムスタンプを付与
- ESP32 (cwmc) は micro-ROS 経由で通信しており、時刻同期が不完全な可能性

scan_relay_node で 150ms の遅延を入れているが、タイムスタンプは変更していない。
これにより slam_toolbox が TF lookup する際に、150ms 前の odom→base_link TF を参照する。
**回転中はこの 150ms 分の角度差が点群のズレとして表れる**。

ただし、scan_relay_node のコメントには「タイムスタンプは変更しない（スキャンデータとポーズの整合性を保つ）」
と記載されており、これは slam_toolbox の MessageFilter の TF lookup に対する正しい対策。
問題は、TF が 150ms 前のスキャンタイムスタンプに対して正確に補間できるかどうか。

### デバッグ手順
1. `ros2 topic echo /scan_delayed --field header.stamp` と `ros2 topic echo /odom --field header.stamp` を比較
2. タイムスタンプの差が 150ms 以内であることを確認
3. slam_toolbox の `transform_timeout: 0.5` が十分か確認

### 修正方法
通常は上位の符号問題が解決すれば顕在化しない。
万が一残る場合は `scan_relay_node.py` の `_DELAY_SEC` を調整（0.15 → 0.10 等）。

---

## 原因 7: YDLiDAR ドライバの frame_id 不一致

### 該当ファイル
- YDLiDAR ドライバの設定（submodule 未初期化のため直接確認不可）
- `khr2026_team1_rspi/src/auto_nav/launch/mapping_launch.py:194`

### 問題の詳細
static_transform_publisher は `laser_frame` を child frame に設定している。
YDLiDAR ドライバが `/scan` の `header.frame_id` に `laser_frame` 以外（例: `laser`、`lidar_frame`）を
設定している場合、TF lookup が失敗し、点群が正しく変換されない。

### デバッグ手順
```bash
ros2 topic echo /scan --field header.frame_id --once
```
出力が `laser_frame` でなければ、TF の child_frame_id と一致させる。

### 修正方法
YDLiDAR の launch ファイル or パラメータで `frame_id` を `laser_frame` に設定するか、
static_transform_publisher の `--child-frame-id` を YDLiDAR の出力 frame_id に合わせる。

---

## 推奨デバッグ手順（実機での確認順序）

### Step 1: 符号の確認（原因 1, 3, 4 の切り分け）
```bash
# 全ノード起動（slam_toolbox なしでも可）
ros2 launch auto_nav mapping_launch.py

# 別ターミナルで IMU とオドメトリを同時に監視
ros2 topic echo /imu --field angular_velocity.z &
ros2 topic echo /odom_raw --field twist.twist.angular.z &

# ロボットを手で左回り（反時計回り）にゆっくり回す
# 両方とも正の値 → OK
# どちらか一方が負 → 該当する方を修正
# 両方とも負 → 両方修正
```

### Step 2: EKF 出力の確認
```bash
ros2 topic echo /odom --field twist.twist.angular.z
# 左回りで正の値 → OK
```

### Step 3: LiDAR TF の確認（原因 2 の切り分け）
```bash
# RViz で Fixed Frame = base_link にして /scan を表示
# ロボット前方に障害物を置いて、RViz でも前方に表示されるか確認
```

### Step 4: frame_id の確認（原因 7 の切り分け）
```bash
ros2 topic echo /scan --field header.frame_id --once
# "laser_frame" であること
```

### Step 5: 統合テスト
```bash
# 修正後、RViz で Fixed Frame = odom にして /scan を表示
# ロボットをその場でゆっくり回転
# 点群が壁に固定されていれば修正成功
```

---

## 修正対象ファイル一覧

| ファイル | リポジトリ | 修正内容 |
|---------|-----------|---------|
| `src/auto_nav/src/auto_nav/imu_publisher_node.py` | rspi | gz 符号反転（原因 1） |
| `src/auto_nav/launch/mapping_launch.py` | rspi | LiDAR TF の pitch 修正（原因 2） |
| `src/auto_nav/launch/auto_nav_launch.py` | rspi | LiDAR TF の pitch 修正（原因 2） |
| `src/auto_nav/src/auto_nav/odometry_node.py` | rspi | omega 符号修正（原因 3、必要な場合） |
| `src/auto_nav/config/ekf_params.yaml` | rspi | EKF 設定調整（原因 4、切り分け用） |
| `lib/lsm9ds1_control/src/lsm9ds1_control.cpp` | cwmc | gz 軸補正（原因 5、ESP32 側で修正する場合） |

---

## 重要な注意事項

- 上記の修正は **実機でのデバッグ結果に基づいて適用する**。コードだけでは符号が正しいかどうかは判断できない（モータの配線、IMU の搭載向き、LiDAR の取り付け方に依存する）。
- 修正は **1 つずつ適用してテスト** すること。複数を同時に変更すると原因の切り分けができなくなる。
- 既存の診断ログ（`imu_publisher_node.py:86-93` の `ratio` 出力）を活用すること。
