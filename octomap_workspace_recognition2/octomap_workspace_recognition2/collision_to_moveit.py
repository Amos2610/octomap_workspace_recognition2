import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node


class PublishCollisionObjectToMoveIt(Node):
    def __init__(self):
        super().__init__("collision_to_moveit")
        self.declare_parameter("collision_object_topic", "/collision_objects")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("clear_on_start", False)

        collision_object_topic = self.get_parameter("collision_object_topic").value
        planning_scene_topic = self.get_parameter("planning_scene_topic").value
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            planning_scene_topic,
            10,
        )
        self.sub = self.create_subscription(
            CollisionObject,
            collision_object_topic,
            self._collision_object_cb,
            10,
        )
        if bool(self.get_parameter("clear_on_start").value):
            self.clear_scene()
        self.get_logger().info(
            f"Forwarding {collision_object_topic} into {planning_scene_topic}"
        )

    def _collision_object_cb(self, msg: CollisionObject):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(msg)
        self.planning_scene_pub.publish(scene)

    def clear_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = PublishCollisionObjectToMoveIt()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
