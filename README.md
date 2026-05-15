# octomap_workspace_recognition2

xArm6 + RealSense D435i を用いたワークスペース自動認識パッケージ（ROS2 Humble）。  
3層Octomapで環境を構築し、MoveIt2との統合による障害物回避・対象物認識を行う。

## 概要

- **3層Octomap** による解像度別の環境表現（static / semi_static / dynamic）
- **自律スキャン** でロボットアームを動かしながらOctomapを構築
- **MoveIt2 統合** で CollisionObject を PlanningScene に自動反映
- **YOLO検出** で対象物を認識し AttachedCollisionObject として把持計画に利用

## 前提環境

| 項目 | バージョン |
|------|-----------|
| OS | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble Hawksbill |
| MoveIt2 | 2.5.x |
| ロボット | xArm6 |
| カメラ | Intel RealSense D435i |

## 依存パッケージ

### apt（追加インストール）

```bash
sudo apt-get install -y \
    ros-humble-octomap \
    ros-humble-octomap-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-cv-bridge
```

### octomap_server（ソースビルド必須）

`ros-humble-octomap-server` は apt で提供されていないため、ソースから取得する。

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/OctoMap/octomap_mapping.git

cd ~/ros2_ws
colcon build --packages-select octomap_server
source install/setup.bash
```

または `.repos` ファイルに以下を追記して `vcs import` でまとめて取得することも可能:

```yaml
octomap_mapping:
  type: git
  url: https://github.com/OctoMap/octomap_mapping.git
  version: humble
```

### xarm_utils_cpp（MoveIt2 Python ラッパー）

このパッケージは `xarm_utils_cpp`（pybind11 C++ ラッパー）を通じて MoveIt2 を操作する。
`my_research.repos` に含まれているため `vcs import` で取得済みの場合が多い。

```bash
# repos から取得していない場合
git clone git@github.com:Amos2610/xarm_utils_cpp.git ~/ros2_ws/src/xarm_utils_cpp
```

## ビルド

```bash
cd ~/ros2_ws

# 依存解決
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --symlink-install

# または対象パッケージのみ
colcon build --symlink-install \
    --packages-select octomap_server octomap_workspace_recognition2

source install/setup.bash
```

## 起動

### Octomapレイヤーのみ

```bash
ros2 launch octomap_workspace_recognition2 octomap_layers.launch.py \
    cloud_topic:=/camera/camera/depth/color/points
```

### フルシステム（カメラ + Octomap + MoveIt2連携）

```bash
ros2 launch octomap_workspace_recognition2 full_system.launch.py
```

### 自律ワークスペース認識

```bash
ros2 run octomap_workspace_recognition2 autonomous_recognition
```

## システム構成

```
RealSense D435i
  ↓ /camera/camera/depth/color/points (PointCloud2)
[octomap_server × 3層]
  ↓ /octomap_binary (Octomap)
[octomap_to_moveit]
  ↓ /planning_scene (PlanningScene)
[MoveIt2 move_group]
  ↓
xArm6 実行
```

### 3層Octomap 構成

| Layer | 解像度 | 距離範囲 | 用途 |
|-------|--------|----------|------|
| static | 0.10 m | 2.0〜5.0 m | 環境全体・粗い障害物 |
| semi_static | 0.05 m | 1.0〜2.0 m | 中間領域・棚等 |
| dynamic | 0.02 m | 0〜1.0 m | ロボット直近・詳細 |

## トピック一覧

| トピック | 型 | 方向 | 説明 |
|---|---|---|---|
| `cloud_topic`（引数指定） | PointCloud2 | 入力 | 深度点群（RealSense） |
| `/octomap_binary` | Octomap | 中継 | Octomapバイナリ表現 |
| `/octomap_point_cloud_centers` | PointCloud2 | 中継 | 占有ボクセル中心点群 |
| `/planning_scene` | PlanningScene | 出力 | MoveIt2へのシーン更新 |

## 設定ファイル

スキャン姿勢は `io/config/scan_poses.yaml` で管理:

```yaml
Joint:
  mapping_static:       # 広域スキャン時のジョイント構成リスト
    - [j1, j2, j3, j4, j5, j6]
    - ...
  mapping_semi_static:  # 中間スキャン時のジョイント構成リスト
  mapping_object:       # 対象物検出時のジョイント構成
```

## 移行設計書

ROS1 (`xarm6_octomap_avoidance`) からの移行詳細は [DESIGN.md](DESIGN.md) を参照。

## ライセンス

Apache License 2.0
