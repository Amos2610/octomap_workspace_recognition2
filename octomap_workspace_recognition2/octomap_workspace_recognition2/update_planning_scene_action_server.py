import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Header

from octomap_workspace_recognition2_interfaces.action import UpdatePlanningScene


def _load_octomap_bt(path):
    with open(path, "rb") as f:
        content = f.read()

    data_marker = b"data\n"
    idx = content.find(data_marker)
    if idx == -1:
        raise ValueError(f"Invalid .bt file (no 'data' marker): {path}")

    header_text = content[:idx].decode("ascii", errors="replace")
    binary_data = content[idx + len(data_marker):]

    octomap_id = "OcTree"
    resolution = 0.1
    frame_id = "world"
    for line in header_text.splitlines():
        line = line.strip()
        if line.startswith("id "):
            octomap_id = line[3:].strip()
        elif line.startswith("res "):
            resolution = float(line[4:].strip())

    return octomap_id, resolution, frame_id, binary_data


class UpdatePlanningSceneActionServer(LifecycleNode):
    def __init__(self):
        super().__init__("update_planning_scene_action_server")

    # -----------------------------------------------------------------------
    # Lifecycle transitions
    # -----------------------------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter("map_save_dir", "")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("frame_id", "world")

        scene_topic = (
            self.get_parameter("planning_scene_topic").get_parameter_value().string_value
            or "/planning_scene"
        )
        self._scene_pub = self.create_publisher(PlanningScene, scene_topic, 10)
        self._is_active = False

        action_cg = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            UpdatePlanningScene,
            "update_planning_scene",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_cg,
        )
        self.get_logger().info("UpdatePlanningScene: configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info("UpdatePlanningScene action server active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        self.get_logger().info("UpdatePlanningScene: deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._scene_pub)
        self.get_logger().info("UpdatePlanningScene: cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _get_map_save_dir(self):
        param = self.get_parameter("map_save_dir").get_parameter_value().string_value
        if param:
            return param
        share = get_package_share_directory("octomap_workspace_recognition2")
        return os.path.join(share, "io", "maps")

    def _resolve_map_dir(self, map_id):
        base = self._get_map_save_dir()
        if not os.path.isdir(base):
            return None
        for workspace_id in os.listdir(base):
            ws_path = os.path.join(base, workspace_id)
            if not os.path.isdir(ws_path):
                continue
            for camera_name in os.listdir(ws_path):
                cam_path = os.path.join(ws_path, camera_name)
                if not os.path.isdir(cam_path):
                    continue
                candidate = os.path.join(cam_path, map_id)
                if os.path.isdir(candidate):
                    return candidate
        return None

    # -----------------------------------------------------------------------
    # Action callbacks
    # -----------------------------------------------------------------------

    def _goal_callback(self, _goal_request):
        if not self._is_active:
            self.get_logger().warn("UpdatePlanningScene: not active, rejecting goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info("Cancel requested for update_planning_scene.")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        map_id = (goal.map_id or "").strip()
        clear_existing = goal.clear_existing

        fb = UpdatePlanningScene.Feedback()

        def publish(progress, phase, message):
            fb.progress = float(progress)
            fb.phase = phase
            fb.message = message
            goal_handle.publish_feedback(fb)

        publish(0.0, "loading_map", f"Resolving map_id='{map_id}'...")

        if not map_id:
            goal_handle.abort()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = "map_id is empty."
            return result

        map_dir = self._resolve_map_dir(map_id)
        if map_dir is None:
            goal_handle.abort()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = f"map_id not found: '{map_id}'"
            return result

        publish(0.15, "loading_map", f"Map directory: {map_dir}")

        metadata_path = os.path.join(map_dir, "metadata.json")
        metadata = {}
        if os.path.exists(metadata_path):
            with open(metadata_path) as f:
                metadata = json.load(f)

        override_frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value or "world"
        )
        frame_id = metadata.get("frame_id", override_frame_id) or override_frame_id

        octomap_path = os.path.join(map_dir, "octomap.bt")
        octomap_msg = None
        if os.path.exists(octomap_path) and os.path.getsize(octomap_path) > 0:
            try:
                oct_id, resolution, _, binary_data = _load_octomap_bt(octomap_path)
                octomap_msg = Octomap()
                octomap_msg.header = Header()
                octomap_msg.header.frame_id = frame_id
                octomap_msg.header.stamp = self.get_clock().now().to_msg()
                octomap_msg.binary = True
                octomap_msg.id = oct_id
                octomap_msg.resolution = resolution
                octomap_msg.data = [b if b < 128 else b - 256 for b in binary_data]
                self.get_logger().info(
                    f"Loaded octomap.bt: id={oct_id} res={resolution} bytes={len(binary_data)}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load octomap.bt: {e}")
        else:
            self.get_logger().warn(f"octomap.bt not found or empty: {octomap_path}")

        publish(0.3, "loading_map", "Map data loaded.")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = "Canceled during loading_map."
            return result

        if clear_existing:
            publish(0.4, "clearing", "Clearing existing planning scene...")
            clear_scene = PlanningScene()
            clear_scene.is_diff = True
            self._scene_pub.publish(clear_scene)
            time.sleep(0.2)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = "Canceled during clearing."
            return result

        applied_octomap = False
        if octomap_msg is not None:
            publish(0.6, "applying_octomap", "Applying OctoMap to planning scene...")
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.octomap.header = octomap_msg.header
            scene.world.octomap.octomap = octomap_msg
            self._scene_pub.publish(scene)
            applied_octomap = True
            time.sleep(0.1)
        else:
            publish(0.6, "applying_octomap", "No OctoMap data; skipping.")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = "Canceled during applying_octomap."
            return result

        applied_objects = 0
        collision_path = os.path.join(map_dir, "collision_objects.json")
        if os.path.exists(collision_path):
            publish(0.8, "applying_collision_objects", "Applying collision objects...")
            try:
                with open(collision_path) as f:
                    coll_data = json.load(f)
                applied_objects = len(coll_data)
                self.get_logger().info(f"Collision objects available: {applied_objects} (publishing TBD)")
            except Exception as e:
                self.get_logger().warn(f"Failed to load collision_objects.json: {e}")
        else:
            publish(0.8, "applying_collision_objects", "No collision objects file; skipping.")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = UpdatePlanningScene.Result()
            result.success = False
            result.message = "Canceled during applying_collision_objects."
            return result

        publish(1.0, "done", f"Planning scene updated from map_id='{map_id}'.")
        self.get_logger().info(
            f"UpdatePlanningScene done: map_id={map_id} applied_octomap={applied_octomap}"
        )

        goal_handle.succeed()
        result = UpdatePlanningScene.Result()
        result.success = True
        result.message = (
            f"Applied map_id='{map_id}': octomap={'yes' if applied_octomap else 'no'}, "
            f"objects={applied_objects}"
        )
        result.applied_objects = applied_objects
        result.applied_octomap = applied_octomap
        return result


def main(args=None):
    rclpy.init(args=args)
    node = UpdatePlanningSceneActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.trigger_configure()
    node.trigger_activate()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
