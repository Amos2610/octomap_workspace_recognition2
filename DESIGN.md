# ROS1→ROS2 移行設計書

**プロジェクト**: xarm6_octomap_avoidance → octomap_workspace_recognition2  
**作成日**: 2026-05-15  
**対象**: xArm6 + RealSense D435i を用いたワークスペース自動認識システム

---

## 1. システム概要

### 1.1 目的

RealSense D435iの深度点群から3層Octomapを構築し、xArm6のMoveIt経由の障害物回避・ワークスペース認識を行うシステムをROS1からROS2へ移行する。

### 1.2 処理フロー

```
RealSense D435i
  ↓ /camera/depth/color/points (PointCloud2) /camera_hand/depth/color/points等で複数トピックあるため，*/depth/color/points*を引数やパラメータで指定出来るようにする
[octomap_server × 3層]
  ↓ OccupancyGrid / Octomap
[CollisionObject 変換]
  ↓ PlanningScene
[MoveIt2 モーション計画]
  ↓
xArm6 実行
```

### 1.3 3層 Octomap 構成

| Layer       | 解像度  | 距離範囲   | 更新周期 | 用途                   |
|-------------|---------|------------|----------|------------------------|
| static      | 0.10 m  | 2.0〜5.0 m | 1.0 s    | 環境全体・粗い障害物   |
| semi_static | 0.05 m  | 1.0〜2.0 m | 0.1 s    | 中間領域・棚等         |
| dynamic     | 0.02 m  | 0〜1.0 m   | 0.01 s   | ロボット直近・詳細     |

---

## 2. ROS1 vs ROS2 アーキテクチャ比較

### 2.1 ビルドシステム

| 項目            | ROS1                    | ROS2                        |
|-----------------|-------------------------|-----------------------------|
| ビルドツール    | catkin                  | ament_python                |
| package.xml     | format="2"              | format="3"                  |
| CMake           | catkin_CMakeLists.txt   | 不要 (Pure Python)           |
| セットアップ    | setup.sh + catkin_make  | colcon build                |

### 2.2 Python API 対応表

| ROS1 (rospy)                            | ROS2 (rclpy)                                     |
|-----------------------------------------|--------------------------------------------------|
| `rospy.init_node('name')`               | `rclpy.init()` + `Node('name')`                  |
| `rospy.Subscriber(topic, Type, cb)`     | `node.create_subscription(Type, topic, cb, qos)` |
| `rospy.Publisher(topic, Type)`          | `node.create_publisher(Type, topic, qos)`        |
| `rospy.Rate(hz)` + `rate.sleep()`       | `node.create_timer(period, cb)`                  |
| `rospy.spin()`                          | `rclpy.spin(node)`                               |
| `rospy.get_param('~param', default)`    | `node.declare_parameter('param', default)` + `node.get_parameter('param').value` |
| `rospy.loginfo/warn/err`                | `node.get_logger().info/warn/error`              |
| `rospy.on_shutdown(cb)`                 | `rclpy.get_default_context().on_shutdown(cb)` または `try/finally` |
| `roslib.packages.get_pkg_dir(pkg)`      | `ament_index_python.packages.get_package_share_directory(pkg)` |
| `rospy.Time.now()`                      | `node.get_clock().now()`                         |
| `rospy.Duration(sec)`                   | `rclpy.duration.Duration(seconds=sec)`           |

### 2.3 MoveIt API 対応表

> **注意**: この環境では `moveit_py` は使用しない。代わりに `xarm_utils_cpp`（C++ `MoveGroupInterface` の pybind11 ラッパー）を `xarm_utils_py` として Python から利用する。これは `path_reuse_sm2` および `example_xarm_utils_py` と同じ方式。

#### ロボット動作制御（`xarm_utils_py` 経由）

| ROS1 (moveit_commander)                         | ROS2 (xarm_utils_py)                                      |
|-------------------------------------------------|-----------------------------------------------------------|
| `moveit_commander.init()`                       | `from xarm_utils_py import XArmUtils, Node`               |
| `MoveGroupCommander('xarm6')`                   | `xarm = XArmUtils(node, 'xarm6')`（Node も pybind11 経由）|
| `group.set_joint_value_target(values)`          | `xarm.set_joint_value_target(values)`                     |
| `group.plan()`                                  | `success, plan, duration, err = xarm.plan()`              |
| `group.execute(plan)`                           | `xarm.execute()`（内部で直前の plan を実行）               |
| `group.go(joint_values, wait=True)`             | `set_joint_value_target` + `plan()` + `execute()` の逐次呼び出し |
| `group.set_max_velocity_scaling_factor(0.3)`    | `xarm.set_move_group_parameter('...', 0.3)`（/move_group へのリモートパラメータ設定） |
| `group.set_planning_pipeline('stomp')`          | `xarm.set_planning_pipeline('stomp')`                     |
| `group.get_current_joint_values()`              | `xarm.get_current_joint_values()`                         |
| `group.compute_ik(...)`                         | `xarm.compute_ik(pose)` または `/compute_ik` サービス      |

**基本パターン（`path_reuse_sm2/example_xarm_utils_py` 準拠）**:

```python
from xarm_utils_py import XArmUtils, Node
# Node() の中で rclpy.init() が呼ばれるため、別途呼び出し不要
node = Node("mapping_node")
xarm = XArmUtils(node, "xarm6")

xarm.set_planning_pipeline("ompl")
xarm.set_joint_value_target([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
success, plan, duration, err = xarm.plan()
if success:
    xarm.execute()
```

#### PlanningScene / CollisionObject 操作（直接トピックパブリッシュ）

`moveit_commander.PlanningSceneInterface` に相当する仕組みは ROS2 でも `/planning_scene` トピックへの直接パブリッシュで実現できる（`MoveGroupInterface` が内部で購読している）。

| ROS1 (moveit_commander)                         | ROS2 (直接 moveit_msgs パブリッシュ)                       |
|-------------------------------------------------|-----------------------------------------------------------|
| `PlanningSceneInterface()`                      | `create_publisher(PlanningScene, '/planning_scene', 10)`  |
| `scene.add_box(name, ...)`                      | `PlanningScene(is_diff=True)` に `CollisionObject` を追加してパブリッシュ |
| `scene.remove_world_object(name)`               | `CollisionObject(operation=REMOVE)` を含む `PlanningScene` をパブリッシュ |
| `CollisionObject` + `AttachedCollisionObject`   | 同構造の `moveit_msgs.msg` を使用（ROS1と互換）            |
| Octomap を PlanningScene に追加                 | `PlanningScene.world.octomap` に設定してパブリッシュ       |

### 2.4 TF API 対応表

| ROS1                              | ROS2                                              |
|-----------------------------------|---------------------------------------------------|
| `tf.TransformListener()`          | `tf2_ros.Buffer()` + `TransformListener(buf, node)` |
| `tf.TransformBroadcaster()`       | `tf2_ros.TransformBroadcaster(node)`              |
| `listener.lookupTransform(t, s)`  | `buf.lookup_transform(t, s, rclpy.time.Time())`   |
| `tf.transformPointCloud(frame, pc)` | `do_transform_cloud()` (tf2_sensor_msgs)        |
| `tf.StampedTransform`             | `geometry_msgs.msg.TransformStamped`              |

### 2.5 Launch システム

| ROS1 (XML)                              | ROS2 (Python)                                       |
|-----------------------------------------|-----------------------------------------------------|
| `<launch>` タグ                         | `generate_launch_description()` 関数               |
| `<node pkg="" type="" name="">`         | `Node(package='', executable='', name='')`          |
| `<param name="" value="">`              | `parameters=[{'name': value}]`                      |
| `<arg name="" default="">`              | `DeclareLaunchArgument('name', default_value='')`   |
| `<remap from="" to="">`                 | `remappings=[('from', 'to')]`                       |
| `<include file="">`                     | `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` |
| `<group ns="">`                         | `PushRosNamespace('ns')`                            |

### 2.6 QoS の扱い

ROS2ではSubscriberとPublisherにQoS設定が必要。センサーデータには以下を推奨：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

---

## 3. コンポーネント別移行計画

### 3.1 移行対象ファイル一覧

| ROS1 ファイル                                       | ROS2 移行先                                            | 優先度 | 状態     |
|-----------------------------------------------------|--------------------------------------------------------|--------|----------|
| `launch/layers_octomap.launch`                      | `launch/octomap_layers.launch.py`                      | 高     | ✅ 完了  |
| `io/param.yaml`                                     | `io/config/scan_poses.yaml`                            | 高     | ✅ 完了  |
| `scripts/node/mapping_move.py`                      | `octomap_workspace_recognition2/mapping_move.py`       | 高     | ✅ 完了  |
| `scripts/node/convert_octomap_to_collision_object.py` | `octomap_workspace_recognition2/convert_octomap.py`  | 高     | ✅ 完了  |
| `scripts/node/publish_octomap_to_moveit.py`         | `octomap_workspace_recognition2/octomap_to_moveit.py`  | 高     | ✅ 完了  |
| `scripts/node/publish_collision_object_to_moveit.py`| `octomap_workspace_recognition2/collision_to_moveit.py`| 高     | ✅ 完了  |
| `scripts/autonomous_workspace_recognition.py`       | `octomap_workspace_recognition2/autonomous_recognition.py` | 高 | ✅ 完了  |
| `scripts/node/yolo_to_collision_object.py`          | `octomap_workspace_recognition2/yolo_detection.py`     | 中     | ✅ 初期実装 |
| `launch/full_system.launch`                         | `launch/full_system.launch.py`                         | 中     | ✅ 完了  |
| `launch/xarm6_bringup.launch`                       | `launch/xarm6_bringup.launch.py`                       | 中     | ✅ 完了  |

### 3.2 mapping_move.py の移行

**役割**: YAMLからジョイント目標値を読み込み、MoveIt2で順次実行する。

**主な変更点**:
- `moveit_commander.MoveGroupCommander` → `xarm_utils_py.XArmUtils`（`path_reuse_sm2` と同じパターン）
- `roslib.packages.get_pkg_dir()` → `ament_index_python.packages.get_package_share_directory()`
- `rospy.get_param()` → YAML直読み（設定ファイルパスは `get_package_share_directory` で解決）
- ROS1では `rospy.init_node` + 通常 `Node`; ROS2では `xarm_utils_py.Node`（内部で `rclpy.init()` 済み）

**移行後クラス設計**:

```python
# octomap_workspace_recognition2/mapping_move.py
import yaml
import os
from xarm_utils_py import XArmUtils, Node
from ament_index_python.packages import get_package_share_directory


class MoveItMapping:
    def __init__(self, node: Node, xarm: XArmUtils, config_path: str):
        self.node = node
        self.xarm = xarm

        with open(config_path) as f:
            self.config = yaml.safe_load(f)

    def execute_poses(self, pose_list_key: str, pipeline: str = "ompl") -> bool:
        poses = self.config["Joint"][pose_list_key]
        self.xarm.set_planning_pipeline(pipeline)
        for joint_values in poses:
            self.xarm.set_joint_value_target(joint_values)
            success, _, _, _ = self.xarm.plan()
            if success:
                if not self.xarm.execute():
                    return False
            else:
                return False
        return True


def main():
    node = Node("mapping_move")
    xarm = XArmUtils(node, "xarm6")

    pkg_share = get_package_share_directory("octomap_workspace_recognition2")
    config_path = os.path.join(pkg_share, "io", "config", "scan_poses.yaml")

    mapper = MoveItMapping(node, xarm, config_path)
    mapper.execute_poses("mapping_static")
```

### 3.3 convert_octomap_to_collision_object.py の移行

**役割**: PointCloud2（占有ボクセル中心）を CollisionObject に変換する。

**主な変更点**:
- `sensor_msgs.msg.PointCloud2` の読み込みは変わらず
- `rospy.Subscriber` → `node.create_subscription(PointCloud2, topic, cb, sensor_qos)`
- `rospy.Publisher` → `node.create_publisher(CollisionObjectArray, topic, 10)`
- `rospy.loginfo` → `self.get_logger().info`

**アルゴリズムロジック（変更なし）**:
- グリッドベースのマージング（merge_size 単位でボクセルをグループ化）
- 最小点数フィルタ（min_points）
- ロボット周辺安全フィルタ（safety_radius）
- 最大オブジェクト数制限（max_objects）

**移行後クラス設計**:

```python
# octomap_workspace_recognition2/convert_octomap.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import CollisionObject

class OctoMap2CollisionObject(Node):
    def __init__(self):
        super().__init__('convert_octomap_to_co')
        self.declare_parameter('source_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('merge_size', 0.15)
        self.declare_parameter('min_points', 5)
        self.declare_parameter('max_objects', 500)
        self.declare_parameter('safety_radius', 0.15)
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('source_topic').value,
            self.callback, sensor_qos)
        self.pub = self.create_publisher(
            CollisionObject, '~/collision_objects', 10)
```

### 3.4 publish_octomap_to_moveit.py の移行

**役割**: Octomap を MoveIt の PlanningScene に継続的に同期する。

**主な変更点**:
- `moveit_commander.PlanningSceneInterface` → 不要。`/planning_scene` トピックに直接パブリッシュ（`MoveGroupInterface` が購読している）
- `rospy.Timer` → `node.create_timer(period, cb)`
- シャットダウン処理: `try/finally` でシーン削除パブリッシュ
- `rospy.Subscriber` → `node.create_subscription(..., sensor_qos)`

**移行後クラス設計**:

```python
# octomap_workspace_recognition2/octomap_to_moveit.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene


class PublishOctomapToMoveIt(Node):
    def __init__(self):
        super().__init__("octomap_to_moveit")
        self.declare_parameter("source_topic", "/octomap_binary")
        self.declare_parameter("update_rate", 1.0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10)

        self.sub = self.create_subscription(
            Octomap,
            self.get_parameter("source_topic").value,
            self._octomap_cb, sensor_qos)

        rate = self.get_parameter("update_rate").value
        self.timer = self.create_timer(rate, self._publish_scene)
        self.latest_octomap = None

    def _octomap_cb(self, msg: Octomap):
        self.latest_octomap = msg

    def _publish_scene(self):
        if self.latest_octomap is None:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.octomap.header = self.latest_octomap.header
        scene.world.octomap.octomap = self.latest_octomap
        self.planning_scene_pub.publish(scene)

    def clear_scene(self):
        """シャットダウン時にOctomapをクリアする."""
        scene = PlanningScene()
        scene.is_diff = True
        # 空の OctomapWithPose を送ることでクリア
        self.planning_scene_pub.publish(scene)


def main():
    rclpy.init()
    node = PublishOctomapToMoveIt()
    try:
        rclpy.spin(node)
    finally:
        node.clear_scene()
        node.destroy_node()
        rclpy.shutdown()
```

### 3.5 yolo_to_collision_object.py の移行

**役割**: YOLO検出結果を深度画像と組み合わせてCollisionObjectに変換する。

**主な変更点**:
- `message_filters.ApproximateTimeSynchronizer` → ROS2版の同名クラス（互換性あり）
- `cv_bridge.CvBridge` → 同名クラス（ROS2対応済）
- `tf.TransformListener` → `tf2_ros.Buffer` + `TransformListener`（`path_reuse_sm2/core/xarm_utils.py` の `XArmRobotUtils` と同パターン）
- `rospy.ServiceProxy('/clear_bbx', ...)` → `node.create_client(BoundingBoxQuery, '/clear_bbx')` + 非同期呼び出し
- CollisionObject送信: `/planning_scene` トピックへの直接パブリッシュ（section 3.4 と同様）

**ROS2サービスコール（非同期）**:

```python
# ROS1
rospy.ServiceProxy('/clear_bbx', BoundingBoxQuery)(req)

# ROS2（path_reuse_sm2/core/xarm_utils.py の compute_ik と同パターン）
future = self.clear_client.call_async(req)
rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
result = future.result()
```

**TF 座標変換（`XArmRobotUtils._transform_pose_to_base` と同パターン）**:

```python
# tf2_ros.Buffer.transform() で camera_frame → map への変換
transformed = self._tf_buffer.transform(
    pose_stamped, "map",
    timeout=rclpy.duration.Duration(seconds=0.5)
)
```

### 3.6 autonomous_workspace_recognition.py の移行

**役割**: mapping_static → mapping_semi_static → mapping_object の3段階オーケストレーション。

**主な変更点**:
- `rospy.init_node` → `xarm_utils_py.Node`（内部で `rclpy.init()` 済み）
- `MoveItMapping` を `xarm_utils_py.XArmUtils` ベースで実装
- `path_reuse_sm2` の `XArmUtilsWrapper` パターンを参考にシングルトン化
- Octomap待機: `/octomap_binary` トピック購読でタイムアウト付き待機

**移行後の推奨アーキテクチャ**:

```python
# octomap_workspace_recognition2/autonomous_recognition.py
from xarm_utils_py import XArmUtils, Node
from octomap_workspace_recognition2.mapping_move import MoveItMapping
# ... OctoMap2CollisionObject, PublishOctomapToMoveIt をインポート

def main():
    node = Node("autonomous_recognition")
    xarm = XArmUtils(node, "xarm6")
    mapper = MoveItMapping(node, xarm, config_path)
    scene_pub = PublishOctomapToMoveIt(node)

    # Phase 1: 広域スキャン
    mapper.execute_poses("mapping_static", pipeline="ompl")
    # Octomap 構築待機 → CollisionObject 変換 → /planning_scene へ送信

    # Phase 2: 中間スキャン
    mapper.execute_poses("mapping_semi_static", pipeline="stomp")

    # Phase 3: 対象物スキャン（YOLO連携）
    mapper.execute_poses("mapping_object", pipeline="ompl")
```

**設計上の注意**:
- `xarm_utils_py.Node` は `rclpy.init()` を内部で呼ぶため、`rclpy.init()` を別途呼ばない
- 複数のノードクラスは同一プロセスで動かすか、launch ファイルで別プロセス起動するかを選択
- 別プロセス起動の場合: `launch/autonomous_recognition.launch.py` で各ノードを `Node(...)` として記述

---

## 4. Launch ファイル移行計画

### 4.1 full_system.launch.py 設計

```
full_system.launch.py
  ├── realsense2_camera (ROS2ドライバ)
  ├── octomap_layers.launch.py (インクルード)
  ├── static_transform_publisher (map↔world, gripper↔camera)
  ├── octomap_to_moveit (各層ごと)
  └── rviz2
```

**ROS2 RealSense ドライバの変更点**:

| ROS1                               | ROS2                                      |
|------------------------------------|-------------------------------------------|
| `realsense2_camera` pkg            | `realsense2_camera` (ROS2版)              |
| `<node name="camera" ...>`         | `Node(package='realsense2_camera', executable='realsense2_camera_node')` |
| `<param name="enable_color" ...>`  | `parameters=[{'enable_color': True}]`     |

### 4.2 xarm6_bringup.launch.py 設計

```
xarm6_bringup.launch.py
  ├── xarm_ros2 bringup launch
  ├── move_group (MoveIt2)
  │   └── sensors_3d.yaml (Octomap plugin)
  └── rviz2 (オプション)
```

**sensors_3d.yaml (MoveIt2用)**:

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 5.0
    padding_offset: 0.1
    padding_scale: 1.0
    max_update_rate: 1.0
    filtered_cloud_topic: /filtered_cloud
```

---

## 5. パッケージ依存関係の更新

### 5.1 package.xml に追加すべき依存

```xml
<!-- 現在 -->
<depend>rclpy</depend>
<exec_depend>octomap_server</exec_depend>

<!-- 追加が必要 -->
<exec_depend>xarm_utils_cpp</exec_depend>        <!-- xarm_utils_py の提供元 C++ パッケージ -->
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>tf2_sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>message_filters</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>octomap_msgs</depend>
<depend>moveit_msgs</depend>
<depend>realsense2_camera</depend>               <!-- ROS2版 -->
```

> `moveit_py` は使用しない。MoveIt 操作は `xarm_utils_cpp` の pybind11 ラッパー（`xarm_utils_py`）経由で行う。

### 5.2 Python依存 (requirements.txt)

```
numpy
scipy
pyyaml
ultralytics   # YOLO11
opencv-python
ament-index-python
```

### 5.3 setup.py の entry_points 追加

```python
entry_points={
    'console_scripts': [
        'mapping_move = octomap_workspace_recognition2.mapping_move:main',
        'convert_octomap = octomap_workspace_recognition2.convert_octomap:main',
        'octomap_to_moveit = octomap_workspace_recognition2.octomap_to_moveit:main',
        'collision_to_moveit = octomap_workspace_recognition2.collision_to_moveit:main',
        'autonomous_recognition = octomap_workspace_recognition2.autonomous_recognition:main',
        'yolo_detection = octomap_workspace_recognition2.yolo_detection:main',
    ],
},
```

---

## 6. 実装フェーズ

### Phase 1: 基盤整備（優先度：高）

1. `package.xml` の依存関係を更新
2. `setup.py` に entry_points を追加
3. `mapping_move.py` をROS2化
4. `convert_octomap.py` をROS2化

### Phase 2: MoveIt2統合（優先度：高）

5. `octomap_to_moveit.py` をROS2化
6. `collision_to_moveit.py` をROS2化
7. `autonomous_recognition.py` をROS2化
8. `full_system.launch.py` を作成

### Phase 3: 高度機能（優先度：中）

9. `yolo_detection.py` をROS2化（YOLO + TF2 + Depth融合）
10. `xarm6_bringup.launch.py` を作成

### Phase 4: テスト・検証（優先度：高）

11. ユニットテスト（各Nodeのテスト）
12. シミュレーション統合テスト（Gazebo + xArm6）
13. 実機動作確認（xArm6 + RealSense D435i）

---

## 7. 環境セットアップ・インストール手順

### 7.1 前提環境

| 項目 | バージョン |
|------|-----------|
| OS | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble Hawksbill |
| Docker ベースイメージ | `nvidia/opengl:1.0-glvnd-runtime-ubuntu22.04` |
| Python | 3.10 |

### 7.2 apt パッケージ

Dockerfile に追加が必要なもの（現状未インストールのものを含む）:

```bash
# Octomap ライブラリ本体とメッセージ定義
sudo apt-get install -y \
    ros-humble-octomap \
    ros-humble-octomap-msgs

# octomap_server は apt で提供されていない（要ソースビルド、7.3参照）

# TF2 関連（通常は ros-humble-desktop に同梱）
sudo apt-get install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs

# OpenCV / cv_bridge
sudo apt-get install -y \
    ros-humble-cv-bridge

# message_filters（ros-humble-desktop に同梱）
sudo apt-get install -y \
    ros-humble-message-filters
```

### 7.3 octomap_server のソースビルド

`ros-humble-octomap-server` は apt では **提供されていない** ため、ソースから追加する。

**`.repos` ファイルへの追記**（`my_research.repos` または `ai_agent_demo.repos`）:

```yaml
repositories:
  # --- 既存エントリ ---
  octomap_mapping:
    type: git
    url: https://github.com/OctoMap/octomap_mapping.git
    version: ros2
  octomap_workspace_recognition2:
    type: git
    url: git@github.com:Amos2610/octomap_workspace_recognition2.git
    version: main
```

**取得・ビルド**:

```bash
cd ~/ros2_ws/src
vcs import < my_research.repos          # または
git clone -b ros2 https://github.com/OctoMap/octomap_mapping.git

cd ~/ros2_ws
colcon build --packages-select octomap_server
```

### 7.4 RealSense ドライバ

`realsense-ros` はソースビルド済み（`realsense-ros` ディレクトリが `.repos` に含まれている）。  
`librealsense2` は以下の apt で提供されており、Dockerfile に設定済み:

```bash
# RealSense 公式リポジトリ追加（Dockerfile 済み）
sudo apt-get install -y \
    librealsense2-dev \
    librealsense2-utils
```

### 7.5 Python パッケージ

```bash
# 必須（Dockerfile 済み）
# ultralytics (YOLO11), opencv-python, numpy, scipy はインストール済み

# 追加インストール不要のもの
# pyyaml, ament-index-python は ros-humble-desktop に同梱
```

### 7.6 ワークスペースのビルド手順

```bash
cd ~/ros2_ws

# 依存解決
rosdep install --from-paths src --ignore-src -r -y

# 全ビルド
colcon build --symlink-install

# または対象パッケージのみ
colcon build --symlink-install \
    --packages-select octomap_workspace_recognition2 octomap_server

source install/setup.bash
```

---

## 8. README に記載する内容

README.md は以下の構成で整備する。

### 8.1 README.md 構成案

```markdown
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
| OS | Ubuntu 22.04 |
| ROS2 | Humble Hawksbill |
| MoveIt2 | 2.5.x |
| ロボット | xArm6 |
| カメラ | Intel RealSense D435i |

## 依存パッケージ

### apt（初回セットアップ）

\`\`\`bash
sudo apt-get install -y \
    ros-humble-octomap \
    ros-humble-octomap-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-cv-bridge
\`\`\`

### octomap_server（ソースビルド必須）

\`\`\`bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/OctoMap/octomap_mapping.git
cd ~/ros2_ws
colcon build --packages-select octomap_server
\`\`\`

### xarm_utils_cpp（MoveIt2 Python ラッパー）

このパッケージは `xarm_utils_cpp`（pybind11 C++ ラッパー）を通じて MoveIt2 を操作する。
`.repos` ファイルに含まれているため `vcs import` で取得可能。

## ビルド

\`\`\`bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select octomap_workspace_recognition2
source install/setup.bash
\`\`\`

## 起動

\`\`\`bash
# Octomapレイヤーのみ起動
ros2 launch octomap_workspace_recognition2 octomap_layers.launch.py \
    cloud_topic:=/camera/camera/depth/color/points

# フルシステム起動（カメラ + Octomap + MoveIt2連携）
ros2 launch octomap_workspace_recognition2 full_system.launch.py

# 自律ワークスペース認識
ros2 run octomap_workspace_recognition2 autonomous_recognition
\`\`\`

## システム構成

\`\`\`
RealSense D435i
  ↓ /camera/camera/depth/color/points (PointCloud2)
[octomap_server × 3層]
  ↓ /octomap_binary (Octomap)
[octomap_to_moveit]
  ↓ /planning_scene (PlanningScene)
[MoveIt2 move_group]
  ↓
xArm6 実行
\`\`\`

## トピック一覧

| トピック | 型 | 方向 | 説明 |
|---|---|---|---|
| `cloud_topic`（引数指定） | PointCloud2 | 入力 | 深度点群（RealSense） |
| `/octomap_binary` | Octomap | 中継 | Octomap バイナリ表現 |
| `/octomap_point_cloud_centers` | PointCloud2 | 中継 | 占有ボクセル中心点群 |
| `/planning_scene` | PlanningScene | 出力 | MoveIt2 へのシーン更新 |
| `/collision_object` | CollisionObject | 出力 | 個別障害物（オプション） |

## パラメータ

主要パラメータは `io/config/scan_poses.yaml` で管理:

- `Joint.mapping_static`: 広域スキャン時のジョイント構成リスト
- `Joint.mapping_semi_static`: 中間スキャン時のジョイント構成リスト
- `Joint.mapping_object`: 対象物検出時のジョイント構成

## ライセンス

Apache License 2.0
```

### 8.2 README.md と DESIGN.md の使い分け

| ドキュメント | 対象読者 | 内容 |
|---|---|---|
| `README.md` | 使用者・開発開始者 | セットアップ手順、起動コマンド、トピック一覧 |
| `DESIGN.md` | 開発者・移行担当者 | ROS1→ROS2の詳細対応表、設計方針、実装フェーズ |

---

## 9. 既知の移行リスクと対処

### 9.1 MoveIt2 Python API の変更

**リスク**: `moveit_commander` はROS2に存在しない。`moveit_py` は使えない（本環境未対応）。  
**対処**: `xarm_utils_cpp` の pybind11 ラッパー（`xarm_utils_py`）を使用。`path_reuse_sm2` の実装が参照実装。

### 9.2 octomap_server の ROS2対応

**リスク**: `octomap_server` が apt で提供されておらず、ソースビルドが必要。  
**対処**: `.repos` ファイルに `octomap_mapping` (ros2 branch) を追加してビルド（セクション 7.3 参照）。  
ROS1版と一部パラメータ名が異なる（`octomap_layers.launch.py` で対応済み）。

### 9.3 RealSense ROS2 ドライバのトピック名変更

**リスク**: ROS2版 `realsense2_camera` のトピック名が変更されている。  
**対処**: launch ファイルの `cloud_topic` 引数で変更可能。実機では確認済み。  
- 変更前（ROS1）: `/camera/depth/color/points`  
- 変更後（ROS2）: `/camera/camera/depth/color/points`（ネームスペース追加）

### 9.4 xarm_ros2 の launch インターフェース

**リスク**: `xarm_ros2` のlaunchファイルのインターフェースが `xarm_ros`（ROS1）と異なる。  
**対処**: `xarm_ros2` のlaunchファイルの引数を確認してから `xarm6_bringup.launch.py` を作成する。  
`path_reuse_sm2/launch/task.launch.py` が参照実装として使える。

### 9.5 QoS 不一致

**リスク**: ROS2でパブリッシャーとサブスクライバーのQoSが一致しないと通信できない。  
**対処**: センサーデータには `BEST_EFFORT`、制御データには `RELIABLE` を使い分ける。  
デバッグ: `ros2 topic info -v <topic>` で QoS を確認する。

### 9.6 TF2 の時刻同期

**リスク**: RealSense の点群タイムスタンプと TF バッファのタイムアウトが不一致でエラーになる。  
**対処**（`path_reuse_sm2/core/xarm_utils.py` の `_transform_pose_to_base` と同パターン）:
```python
transformed = tf_buffer.transform(
    pose_stamped, "map",
    timeout=rclpy.duration.Duration(seconds=0.5)
)
```

---

## 10. ディレクトリ構成（移行後）

```
octomap_workspace_recognition2/
├── package.xml                              ← 依存更新
├── setup.py                                 ← entry_points追加
├── setup.cfg
├── io/
│   └── config/
│       └── scan_poses.yaml                  ← 既存（変更なし）
├── launch/
│   ├── octomap_layers.launch.py             ← 既存（完了）
│   ├── full_system.launch.py                ← 新規作成
│   └── xarm6_bringup.launch.py             ← 新規作成
├── octomap_workspace_recognition2/
│   ├── __init__.py
│   ├── mapping_move.py                      ← 新規作成 (ROS2化)
│   ├── convert_octomap.py                   ← 新規作成 (ROS2化)
│   ├── octomap_to_moveit.py                 ← 新規作成 (ROS2化)
│   ├── collision_to_moveit.py               ← 新規作成 (ROS2化)
│   ├── autonomous_recognition.py            ← 新規作成 (ROS2化)
│   └── yolo_detection.py                    ← 新規作成 (ROS2化)
├── config/
│   └── sensors_3d.yaml                      ← 新規作成 (MoveIt2用)
├── rviz/
│   └── xarm6_octomap.rviz                   ← RViz2形式で作成
├── resource/
│   └── octomap_workspace_recognition2
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## 11. 参考: ハードコード値一覧

以下は移行後もパラメータとして引き継ぐ値：

| パラメータ           | 値                           | 用途                          |
|----------------------|------------------------------|-------------------------------|
| cloud_topic          | /camera/depth/color/points   | 点群入力トピック               |
| frame_id             | map                          | Octomapの基準座標系           |
| merge_size (static)  | 0.15 m                       | CollisionObject グリッド幅    |
| merge_size (semi)    | 0.06 m                       | semi-static グリッド幅        |
| min_points           | 5〜30                        | CO最小ボクセル数               |
| max_objects          | 500                          | 最大CO数                      |
| safety_radius        | 0.15 m                       | ロボット周辺クリア半径         |
| velocity_scale       | 0.3                          | MoveIt最大速度スケール         |
| accel_scale          | 0.15                         | MoveIt最大加速度スケール       |
| yolo_model           | yolo11n.pt                   | YOLOモデルファイル             |
| yolo_conf            | 0.50                         | YOLO信頼度閾値                |
| roi_shrink           | 0.85                         | ROI縮小率                     |
| depth_band           | 0.08 m                       | 深度フィルタ幅（中央値±）      |
| clear_expand         | 0.012 m                      | OctoMapクリア範囲拡張          |
| robot_ip             | 192.168.1.195                | xArm6 実機IPアドレス          |
| attach_link          | link_eef                     | グリッパー先端リンク名        |
