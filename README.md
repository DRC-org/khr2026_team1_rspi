# khr2026_team1_rspi

関西春ロボコン 2026 チーム 1 RSPI（Raspberry Pi Interface）

LiDAR や CWMC からのセンサ情報などを収集し、自己位置推定や経路計画を行います。GUI なども提供します。

## 開発環境の構築

### 必要なソフトウェア

- VS Code
- WSL2
- ROS 2 Kilted Kaiju（WSL 上にインストール）

### セットアップ

#### 1. WSL2 のセットアップ

WSL2 上に Ubuntu をインストールします。

```sh
wsl --install
```

インストールできたら、Windows ターミナルから Ubuntu に接続します。

```sh
sudo apt update
sudo apt upgrade
```

#### 2. ROS 2 Kilted Kaiju のインストール

以下のコマンドをすべて実行し、ROS 2 Kilted Kaiju をインストールします。

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt install ros-kilted-desktop
source /opt/ros/kilted/setup.bash
echo -e "\n# ROS 2\nsource /opt/ros/kilted/setup.bash" >> ~/.bashrc
```

#### 3. Build Essential のインストール

```sh
sudo apt update
sudo apt install build-essential
```

## メモ

### uv を使いながら Python のパッケージを作る

#### つくるとき

```sh
cd src
ros2 pkg create pkg_name
cd pkg_name
uv init . --lib --python-preference only-system
uv venv --system-site-packages
```

- `uv init`
  - `--lib`: ライブラリ形式で初期化する
  - `--python-preference only-system`: システムの Python を使う
- `uv venv --system-site-packages`: システムのパッケージを使えるようにする（`rclpy` など）

#### ビルドするとき

```sh
colcon build --packages-select pkg_name --symlink-install
```

`--symlink-install` をつけると、シンボリックリンクとしてインストールされるため、ソースファイルの変更をビルドなしで反映できるようになる

#### 実行するとき

```sh
source install/setup.bash
cd src/pkg_name
source .venv/bin/activate
ros2 run pkg_name file_name.py
```
