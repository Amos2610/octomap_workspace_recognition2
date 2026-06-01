import math

import message_filters
import rclpy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive


class YoloDetectionToCollisionObject(Node):
    def __init__(self):
        super().__init__("yolo_detection")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("object_size", [0.05, 0.05, 0.05])

        self.bridge = CvBridge()
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.scene_pub = self.create_publisher(
            PlanningScene,
            self.get_parameter("planning_scene_topic").value,
            10,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            10,
        )

        image_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("image_topic").value,
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("depth_topic").value,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub],
            queue_size=5,
            slop=0.1,
        )
        self.sync.registerCallback(self._callback)

        try:
            from ultralytics import YOLO
            self.model = YOLO(self.get_parameter("model").value)
        except Exception as exc:
            self.model = None
            self.get_logger().error(f"Failed to load YOLO model: {exc}")

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _callback(self, image_msg, depth_msg):
        if self.model is None or self.camera_info is None:
            return

        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        results = self.model.predict(
            image,
            conf=float(self.get_parameter("confidence").value),
            verbose=False,
        )

        for result_index, result in enumerate(results):
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box_index, box in enumerate(boxes):
                xyxy = box.xyxy[0].cpu().numpy()
                pose = self._box_center_pose(xyxy, depth, depth_msg.header)
                if pose is None:
                    continue
                self._publish_collision_object(
                    pose,
                    f"yolo_{result_index}_{box_index}",
                )

    def _box_center_pose(self, xyxy, depth, header):
        x1, y1, x2, y2 = [int(value) for value in xyxy]
        cx = max(0, min(depth.shape[1] - 1, (x1 + x2) // 2))
        cy = max(0, min(depth.shape[0] - 1, (y1 + y2) // 2))
        z = float(depth[cy, cx])
        if not math.isfinite(z) or z <= 0.0:
            return None
        if str(depth.dtype) not in ("float32", "float64"):
            z *= 0.001

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        px = self.camera_info.k[2]
        py = self.camera_info.k[5]

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = (cx - px) * z / fx
        pose.pose.position.y = (cy - py) * z / fy
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(
                pose,
                self.get_parameter("target_frame").value,
                timeout=Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return None

    def _publish_collision_object(self, pose_stamped, object_id):
        size = [float(value) for value in self.get_parameter("object_size").value]
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        collision_object = CollisionObject()
        collision_object.header = pose_stamped.header
        collision_object.id = object_id
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose_stamped.pose)
        collision_object.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)
        self.scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionToCollisionObject()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
