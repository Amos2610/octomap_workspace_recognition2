import math
from collections import defaultdict

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _xyz(point):
    try:
        return float(point[0]), float(point[1]), float(point[2])
    except (TypeError, ValueError, IndexError):
        return float(point.x), float(point.y), float(point.z)


class OctoMap2CollisionObject(Node):
    def __init__(self):
        super().__init__("convert_octomap_to_collision_object")
        self.declare_parameter("source_topic", "/octomap_point_cloud_centers")
        self.declare_parameter("collision_object_topic", "/collision_objects")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("merge_size", 0.15)
        self.declare_parameter("min_points", 5)
        self.declare_parameter("max_objects", 500)
        self.declare_parameter("safety_radius", 0.15)
        self.declare_parameter("object_prefix", "octomap")

        source_topic = self.get_parameter("source_topic").value
        output_topic = self.get_parameter("collision_object_topic").value
        self.sub = self.create_subscription(
            PointCloud2,
            source_topic,
            self.callback,
            sensor_qos(),
        )
        self.pub = self.create_publisher(CollisionObject, output_topic, 10)
        self.get_logger().info(f"Converting {source_topic} to {output_topic}")

    def callback(self, msg: PointCloud2):
        merge_size = float(self.get_parameter("merge_size").value)
        min_points = int(self.get_parameter("min_points").value)
        max_objects = int(self.get_parameter("max_objects").value)
        safety_radius = float(self.get_parameter("safety_radius").value)
        object_prefix = self.get_parameter("object_prefix").value
        frame_id = self.get_parameter("frame_id").value or msg.header.frame_id

        groups = defaultdict(list)
        for point in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x, y, z = _xyz(point)
            if safety_radius > 0.0 and math.hypot(x, y) < safety_radius:
                continue
            key = (
                math.floor(x / merge_size),
                math.floor(y / merge_size),
                math.floor(z / merge_size),
            )
            groups[key].append((x, y, z))

        objects = []
        for key, points in groups.items():
            if len(points) < min_points:
                continue
            objects.append((len(points), key, self._make_object(
                points,
                frame_id,
                f"{object_prefix}_{key[0]}_{key[1]}_{key[2]}",
            )))

        objects.sort(key=lambda item: item[0], reverse=True)
        for _, _, collision_object in objects[:max_objects]:
            self.pub.publish(collision_object)

        self.get_logger().debug(f"Published {min(len(objects), max_objects)} objects")

    def _make_object(self, points, frame_id, object_id):
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            max(max_x - min_x, 0.01),
            max(max_y - min_y, 0.01),
            max(max_z - min_z, 0.01),
        ]

        pose = Pose()
        pose.position.x = (min_x + max_x) * 0.5
        pose.position.y = (min_y + max_y) * 0.5
        pose.position.z = (min_z + max_z) * 0.5
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object


def main(args=None):
    rclpy.init(args=args)
    node = OctoMap2CollisionObject()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
