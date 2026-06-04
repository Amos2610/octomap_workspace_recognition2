import rclpy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PublishOctomapToMoveIt(Node):
    def __init__(self):
        super().__init__("octomap_to_moveit")
        self.declare_parameter("source_topic", "/octomap_binary")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("update_period", 1.0)
        self.declare_parameter("frame_id", "")

        self.latest_octomap = None
        source_topic = self.get_parameter("source_topic").value
        planning_scene_topic = self.get_parameter("planning_scene_topic").value
        update_period = float(self.get_parameter("update_period").value)

        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            planning_scene_topic,
            10,
        )
        self.sub = self.create_subscription(
            Octomap,
            source_topic,
            self._octomap_cb,
            sensor_qos(),
        )
        self.timer = self.create_timer(update_period, self._publish_scene)
        self.get_logger().info(
            f"Publishing {source_topic} into {planning_scene_topic}"
        )

    def _octomap_cb(self, msg: Octomap):
        self.latest_octomap = msg

    def _publish_scene(self):
        if self.latest_octomap is None:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.octomap.header = self.latest_octomap.header
        frame_id = self.get_parameter("frame_id").value
        if frame_id:
            scene.world.octomap.header.frame_id = frame_id
        scene.world.octomap.octomap = self.latest_octomap
        self.planning_scene_pub.publish(scene)

    def clear_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = PublishOctomapToMoveIt()
    try:
        rclpy.spin(node)
    finally:
        node.clear_scene()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
