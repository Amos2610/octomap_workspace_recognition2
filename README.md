# octomap_workspace_recognition2

xArm6 + RealSense D435i を用いたワークスペース自動認識パッケージ（ROS2 Humble）。  
3層Octomapで環境を構築し、MoveIt2との統合による障害物回避・対象物認識を行う。

## 概要

- **3層Octomap** による解像度別の環境表現（static / semi_static / dynamic）
- **自律スキャン** でロボットアームを動かしながらOctomapを構築
- **MoveIt2 統合** で CollisionObject を PlanningScene に自動反映
- **YOLO検出** で対象物を認識し AttachedCollisionObject として把持計画に利用
- **ROS 2 Action** で外部エージェント（RAGシステム等）からスキャン・シーン更新を呼び出せる

## パッケージ構成

| パッケージ | 役割 |
|---|---|
| `octomap_workspace_recognition2` | 実行ノード・Action Server 本体 |
| `octomap_workspace_recognition2_interfaces` | Action / Message 定義のみ |
| `octomap_workspace_recognition2_utils` | エージェント側クライアントユーティリティ |

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
git clone -b ros2 https://github.com/OctoMap/octomap_mapping.git

cd ~/ros2_ws
colcon build --packages-select octomap_server
source install/setup.bash
```

または `.repos` ファイルに以下を追記して `vcs import` でまとめて取得することも可能:

```yaml
octomap_mapping:
  type: git
  url: https://github.com/OctoMap/octomap_mapping.git
  version: ros2
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

# 全ビルド
colcon build --symlink-install

# または octomap 関連3パッケージのみ
colcon build --base-paths src/octomap_workspace_recognition2 \
    --packages-select \
    octomap_workspace_recognition2_interfaces \
    octomap_workspace_recognition2_utils \
    octomap_workspace_recognition2

source install/setup.bash
```

> **注意**: パッケージ構成を変更した場合は `build/` と `install/` 内の対象パッケージディレクトリを削除してからリビルドしてください。

## 起動

### Action Server のみ（推奨：エージェント連携時）

> **注意**: ビルド後は必ずターミナルで `source ~/ros2_ws/install/setup.bash` を実行してから起動すること。  
> ビルド前に source した場合、新規パッケージのPythonパスがシェルに反映されずインポートエラーになる。

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch octomap_workspace_recognition2 octomap_wp_recog_action_servers.launch.py
```

パラメータを指定する場合:

```bash
ros2 launch octomap_workspace_recognition2 octomap_wp_recog_action_servers.launch.py \
    map_save_dir:=/tmp/my_maps \
    known_cameras:=hand_camera
```

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `map_save_dir` | `<package_share>/io/maps` | スキャン結果の保存先ベースディレクトリ |
| `known_cameras` | `hand_camera` | 受け付けるカメラ名（カンマ区切りで複数指定可） |

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

### 個別ノード

```bash
ros2 run octomap_workspace_recognition2 mapping_move --pose-key mapping_static
ros2 run octomap_workspace_recognition2 convert_octomap --ros-args \
    -p source_topic:=/octomap_static/octomap_point_cloud_centers
ros2 run octomap_workspace_recognition2 collision_to_moveit
ros2 run octomap_workspace_recognition2 yolo_detection
```

---

## Action Server インタフェース

### ScanWorkspace

ワークスペースをスキャンし、結果をファイルに保存する。

**Goal**

| フィールド | 型 | 説明 |
|---|---|---|
| `request_id` | string | リクエスト識別子（空でも可） |
| `workspace_id` | string | ワークスペース名（保存ディレクトリ名に使用） |
| `camera_name` | string | 使用カメラ名（空の場合 `hand_camera` に自動設定） |
| `duration_sec` | float64 | スキャン時間（秒） |

**Result**

| フィールド | 型 | 説明 |
|---|---|---|
| `success` | bool | 成否 |
| `message` | string | 詳細メッセージ |
| `map_id` | string | 生成されたマップID |
| `map_path` | string | 保存先ディレクトリ絶対パス |
| `summary_json` | string | スキャン結果サマリ（JSON文字列） |
| `summary` | ScanWorkspaceSummary | スキャン結果サマリ（ROS msg） |

**Feedback**

| フィールド | 型 | 説明 |
|---|---|---|
| `progress` | float32 | 進捗 0.0〜1.0 |
| `phase` | string | 現在フェーズ（`initializing` / `scanning` / `saving` / `done`） |
| `message` | string | 詳細メッセージ |

#### カメラ名の扱い

- `camera_name` が空 → `hand_camera` に自動設定
- `known_cameras` に含まれないカメラ名 → `success=false` でゴール拒否
- 並行スキャン中に新ゴールが来た場合 → `GoalResponse.REJECT`

#### 保存ディレクトリ構造

```
map_save_dir/
  {workspace_id}/
    {camera_name}/
      {map_id}/
        octomap.bt       # OctoMapバイナリ
        summary.json     # スキャン結果サマリ
        metadata.json    # リクエスト・保存メタ情報
```

`map_id` の形式: `{workspace_id}_{camera_name}_{YYYYMMDD}_{HHMMSS}_{suffix}`  
例: `front_workspace_hand_camera_20260601_143012_a7f9`

### UpdatePlanningScene

保存済みスキャン結果をMoveIt PlanningScene に反映する。

**Goal**

| フィールド | 型 | 説明 |
|---|---|---|
| `request_id` | string | リクエスト識別子 |
| `map_id` | string | 反映するスキャン結果の map_id |
| `clear_existing` | bool | 既存のシーン情報を削除してから反映するか |

**Result**

| フィールド | 型 | 説明 |
|---|---|---|
| `success` | bool | 成否 |
| `message` | string | 詳細メッセージ |
| `applied_objects` | int32 | 反映したCollisionObject数 |
| `applied_octomap` | bool | OctoMapを反映したか |

**Feedback フェーズ**

| phase | 説明 |
|---|---|
| `loading_map` | map_id からディレクトリを解決中 |
| `clearing` | 既存シーン情報を削除中 |
| `applying_octomap` | OctoMap を反映中 |
| `applying_collision_objects` | CollisionObject を反映中 |

---

## エージェントからの利用（octomap_workspace_recognition2_utils）

`rag_factory_specific_task_agent` 等のエージェントは `octomap_workspace_recognition2_utils` を通じて Action を呼び出す。

### 基本的な使い方

```python
from octomap_workspace_recognition2_utils.rag_tool import (
    scan_workspace,
    update_planning_scene,
)

# ワークスペースをスキャン
scan_result = scan_workspace(
    workspace_id="front_workspace",
    duration_sec=5.0,
    camera_name="hand_camera",   # 省略すると hand_camera
)

print(scan_result["success"])    # True / False
print(scan_result["map_id"])     # front_workspace_hand_camera_...
print(scan_result["summary"]["obstacle_detected"])

# PlanningScene に反映
if scan_result["success"]:
    scene_result = update_planning_scene(
        map_id=scan_result["map_id"],
        clear_existing=True,
    )
    print(scene_result["applied_octomap"])  # True
```

### 低レイヤークライアントの直接利用

```python
import rclpy
from octomap_workspace_recognition2_utils.scan_workspace_client import ScanWorkspaceClient
from octomap_workspace_recognition2_utils.update_planning_scene_client import UpdatePlanningSceneClient

rclpy.init()

scan_client = ScanWorkspaceClient()
result = scan_client.call(
    workspace_id="front_workspace",
    duration_sec=5.0,
    camera_name="hand_camera",
    timeout_sec=120.0,
)
scan_client.destroy_node()

update_client = UpdatePlanningSceneClient()
result = update_client.call(map_id="...", clear_existing=True)
update_client.destroy_node()

rclpy.shutdown()
```

---

## 動作検証

### 1. ビルド確認

```bash
cd ~/ros2_ws
colcon build --base-paths src/octomap_workspace_recognition2 \
    --packages-select \
    octomap_workspace_recognition2_interfaces \
    octomap_workspace_recognition2_utils \
    octomap_workspace_recognition2
```

すべて `Finished` で終了することを確認する。

### 2. インポート確認

```bash
source ~/ros2_ws/install/setup.bash

# インタフェース
python3 -c "
from octomap_workspace_recognition2_interfaces.action import ScanWorkspace, UpdatePlanningScene
from octomap_workspace_recognition2_interfaces.msg import ScanWorkspaceSummary
print('interfaces OK')
"

# Action Server
python3 -c "
from octomap_workspace_recognition2.scan_workspace_action_server import ScanWorkspaceActionServer
from octomap_workspace_recognition2.update_planning_scene_action_server import UpdatePlanningSceneActionServer
print('action servers OK')
"

# Utils
python3 -c "
from octomap_workspace_recognition2_utils.rag_tool import scan_workspace, update_planning_scene
print('rag_tool OK')
"
```

### 3. Action Server 単体動作テスト

ターミナル1でサーバを起動する:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch octomap_workspace_recognition2 action_servers.launch.py \
    map_save_dir:=/tmp/octomap_test_maps
```

ターミナル2でスキャンゴールを送信する:

```bash
source ~/ros2_ws/install/setup.bash

# ScanWorkspace テスト（正常系）
ros2 action send_goal -f /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{request_id: 'test001', workspace_id: 'front_workspace', camera_name: 'hand_camera', duration_sec: 2.0}"
```

期待する結果:
- フィードバックとして `initializing` → `scanning` → `saving` → `done` が表示される
- `success: true` と `map_id` が返る
- `/tmp/octomap_test_maps/front_workspace/hand_camera/<map_id>/` に3ファイルが生成される

```bash
# 空の camera_name は hand_camera にフォールバック
ros2 action send_goal /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{workspace_id: 'front_workspace', camera_name: '', duration_sec: 1.0}"

# 未知カメラは success=false で返る
ros2 action send_goal /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{workspace_id: 'front_workspace', camera_name: 'unknown_cam', duration_sec: 1.0}"
```

### 4. UpdatePlanningScene テスト

上記スキャン後に返った `map_id` を使用する:

```bash
ros2 action send_goal -f /update_planning_scene \
    octomap_workspace_recognition2_interfaces/action/UpdatePlanningScene \
    "{request_id: 'upd001', map_id: '<上で取得したmap_id>', clear_existing: true}"
```

期待する結果:
- `loading_map` → `clearing` → `applying_octomap` → `applying_collision_objects` → `done` が表示される
- `success: true` が返る

存在しない `map_id` の場合:

```bash
ros2 action send_goal /update_planning_scene \
    octomap_workspace_recognition2_interfaces/action/UpdatePlanningScene \
    "{map_id: 'nonexistent_map_id', clear_existing: false}"
# → success: false が返ることを確認
```

### 5. 保存ファイル確認

```bash
# スキャン後にディレクトリ構造を確認
find /tmp/octomap_test_maps -type f | sort

# summary.json の内容確認
cat /tmp/octomap_test_maps/front_workspace/hand_camera/*/summary.json | python3 -m json.tool

# metadata.json の内容確認
cat /tmp/octomap_test_maps/front_workspace/hand_camera/*/metadata.json | python3 -m json.tool
```

### 6. 並行スキャン拒否テスト

スキャン実行中（`duration_sec` を大きくして）に別ゴールを送信し、`REJECTED` が返ることを確認する:

```bash
# ターミナル A: 長めのスキャン開始
ros2 action send_goal /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{workspace_id: 'ws', camera_name: 'hand_camera', duration_sec: 10.0}" &

# ターミナル B: 並行して別ゴールを送信（即座に REJECTED になる）
ros2 action send_goal /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{workspace_id: 'ws2', camera_name: 'hand_camera', duration_sec: 1.0}"
```

### 7. キャンセルテスト

```bash
# バックグラウンドでゴール送信しゴール ID を取得
ros2 action send_goal /scan_workspace \
    octomap_workspace_recognition2_interfaces/action/ScanWorkspace \
    "{workspace_id: 'ws', camera_name: 'hand_camera', duration_sec: 30.0}" &

# ros2 action cancel でキャンセル（GUIDは上記出力から取得）
ros2 action cancel /scan_workspace <goal-uuid>
```

---

## システム構成

```
RealSense D435i
  ↓ /camera/hand_camera/depth/color/points (PointCloud2)
[octomap_server × 3層]
  ↓ /octomap_binary (Octomap)
[octomap_to_moveit]
  ↓ /planning_scene (PlanningScene)
[MoveIt2 move_group]
  ↓
xArm6 実行
```

### Action ベースの連携フロー

```
rag_factory_specific_task_agent
  ↓ scan_workspace() / update_planning_scene()
[octomap_workspace_recognition2_utils]
  ↓ ROS 2 Action Client
[scan_workspace_action_server]  ←→  [update_planning_scene_action_server]
  ↓ map_save_dir/workspace_id/camera_name/map_id/
  octomap.bt / summary.json / metadata.json
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
| `/collision_objects` | CollisionObject | 出力 | Octomap/YOLOから生成した障害物 |

## Action トピック一覧

| トピック | 型 | 説明 |
|---|---|---|
| `/scan_workspace` | ScanWorkspace | ワークスペーススキャン |
| `/update_planning_scene` | UpdatePlanningScene | PlanningScene 更新 |

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
