import time

from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header


class ObstacleRegistrar:
    """
    OctoMap → /planning_scene への登録ロジックを共有するヘルパー。
    RegisterObstacles, ScanWorkspace, MoveAndScanObstacle の各 Action Server が共用する。
    """

    def __init__(self, node, scene_pub, frame_id: str = "world"):
        self._node = node
        self._scene_pub = scene_pub
        self._frame_id = frame_id

    def apply(self, octomap_msg: Octomap, clear_existing: bool = False) -> bool:
        """
        OctoMap を MoveIt planning_scene に publish する。
        clear_existing=True の場合は先に空 diff でシーンをクリアする。
        """
        if octomap_msg is None:
            self._node.get_logger().warn("ObstacleRegistrar: octomap_msg is None, skipping.")
            return False

        if clear_existing:
            clear_scene = PlanningScene()
            clear_scene.is_diff = True
            self._scene_pub.publish(clear_scene)
            time.sleep(0.2)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.octomap.header = Header()
        scene.world.octomap.header.frame_id = self._frame_id
        scene.world.octomap.header.stamp = self._node.get_clock().now().to_msg()
        scene.world.octomap.octomap = octomap_msg
        self._scene_pub.publish(scene)
        return True
