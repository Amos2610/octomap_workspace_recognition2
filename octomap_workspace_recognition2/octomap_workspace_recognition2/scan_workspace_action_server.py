import json
import math
import os
import threading
import uuid
from datetime import datetime, timezone

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from xarm_utils_py import XArmUtils
from xarm_utils_py import Node as XArmNode

from octomap_workspace_recognition2.mapping_move import MoveItMapping, default_config_path
from octomap_workspace_recognition2.obstacle_registrar import ObstacleRegistrar
from octomap_workspace_recognition2_interfaces.action import ScanWorkspace
from octomap_workspace_recognition2_interfaces.msg import ScanWorkspaceSummary

DEFAULT_CAMERA = "hand_camera"
SCHEMA_VERSION = "scan_workspace_summary.v1"

CAMERA_PROFILES = {
    "hand_camera": {
        "topic": "/camera/hand_camera/depth/color/points",
        "frame_id": "world",
        "octomap_topic": "/octomap_static/octomap_binary",
        "cloud_topic": "/octomap_static/octomap_point_cloud_centers",
    },
}


def _sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _make_map_id(workspace_id, camera_name, request_id):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_ws = workspace_id.replace(" ", "_").replace("/", "_")
    safe_cam = camera_name.replace(" ", "_").replace("/", "_")
    suffix = (request_id[-4:].lower() if request_id else uuid.uuid4().hex[:4])
    return f"{safe_ws}_{safe_cam}_{ts}_{suffix}"


def _save_octomap_bt(octomap_msg, path):
    header = (
        f"# Octomap OcTree binary file\n"
        f"id {octomap_msg.id}\n"
        f"res {octomap_msg.resolution}\n"
        f"data\n"
    ).encode("ascii")
    raw = bytes(v & 0xFF for v in octomap_msg.data)
    with open(path, "wb") as f:
        f.write(header)
        f.write(raw)


def _stats_from_cloud(cloud_msg):
    occupied = 0
    nearest_dist = -1.0
    try:
        pts = list(point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True
        ))
        occupied = len(pts)
        if occupied > 0:
            nearest_dist = min(
                math.sqrt(float(p[0]) ** 2 + float(p[1]) ** 2 + float(p[2]) ** 2)
                for p in pts
            )
    except Exception:
        pass
    return occupied, nearest_dist


class ScanWorkspaceActionServer(LifecycleNode):
    def __init__(self, xarm_node: XArmNode):
        super().__init__("scan_workspace_action_server")
        self._xarm_node = xarm_node

    # -----------------------------------------------------------------------
    # Lifecycle transitions
    # -----------------------------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter("map_save_dir", "")
        self.declare_parameter("known_cameras", DEFAULT_CAMERA)
        self.declare_parameter("move_group", "xarm6")
        self.declare_parameter("config_path", "")
        self.declare_parameter("pose_list_key", "mapping_static")
        self.declare_parameter("per_pose_wait_sec", 3.0)
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("frame_id", "world")

        move_group = (
            self.get_parameter("move_group").get_parameter_value().string_value or "xarm6"
        )
        self._xarm = XArmUtils(self._xarm_node, move_group)

        cfg = self.get_parameter("config_path").get_parameter_value().string_value
        self._config_path = cfg if cfg else default_config_path()
        self._mapper = MoveItMapping(self, self._xarm, self._config_path)

        scene_topic = (
            self.get_parameter("planning_scene_topic").get_parameter_value().string_value
            or "/planning_scene"
        )
        frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value or "world"
        )
        self._scene_pub = self.create_publisher(PlanningScene, scene_topic, 10)
        self._registrar = ObstacleRegistrar(self, self._scene_pub, frame_id)

        self._latest_octomap: Octomap = None
        self._latest_cloud: PointCloud2 = None
        self._octomap_event = threading.Event()
        self._lock = threading.Lock()
        self._scanning = False
        self._is_active = False
        self._octomap_sub = None
        self._cloud_sub = None

        # Action server created once here; destroyed in on_cleanup only.
        # Acceptance controlled by _is_active flag.
        action_cg = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ScanWorkspace,
            "scan_workspace",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_cg,
        )
        self.get_logger().info("ScanWorkspace: configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        sub_cg = MutuallyExclusiveCallbackGroup()
        self._octomap_sub = self.create_subscription(
            Octomap,
            CAMERA_PROFILES[DEFAULT_CAMERA]["octomap_topic"],
            self._on_octomap,
            _sensor_qos(),
            callback_group=sub_cg,
        )
        sub_cg2 = MutuallyExclusiveCallbackGroup()
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            CAMERA_PROFILES[DEFAULT_CAMERA]["cloud_topic"],
            self._on_cloud,
            _sensor_qos(),
            callback_group=sub_cg2,
        )
        self._is_active = True
        self.get_logger().info("ScanWorkspace action server active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        if self._octomap_sub is not None:
            self.destroy_subscription(self._octomap_sub)
            self._octomap_sub = None
        if self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)
            self._cloud_sub = None
        self.get_logger().info("ScanWorkspace: deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._scene_pub)
        self.get_logger().info("ScanWorkspace: cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # -----------------------------------------------------------------------
    # OctoMap / PointCloud callbacks
    # -----------------------------------------------------------------------

    def _on_octomap(self, msg: Octomap):
        self._latest_octomap = msg
        self._octomap_event.set()

    def _on_cloud(self, msg: PointCloud2):
        self._latest_cloud = msg

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _get_map_save_dir(self):
        param = self.get_parameter("map_save_dir").get_parameter_value().string_value
        if param:
            return param
        share = get_package_share_directory("octomap_workspace_recognition2")
        return os.path.join(share, "io", "maps")

    def _get_known_cameras(self):
        raw = self.get_parameter("known_cameras").get_parameter_value().string_value
        cams = [c.strip() for c in raw.split(",") if c.strip()]
        return cams if cams else [DEFAULT_CAMERA]

    def _load_poses(self):
        key = (
            self.get_parameter("pose_list_key").get_parameter_value().string_value
            or "mapping_static"
        )
        with open(self._config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return cfg.get("Joint", {}).get(key, [])

    # -----------------------------------------------------------------------
    # Action callbacks
    # -----------------------------------------------------------------------

    def _goal_callback(self, _goal_request):
        if not self._is_active:
            self.get_logger().warn("ScanWorkspace: not active, rejecting goal.")
            return GoalResponse.REJECT
        with self._lock:
            if self._scanning:
                self.get_logger().warn("Rejecting goal: another scan is running.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        camera_name = (goal.camera_name or "").strip() or DEFAULT_CAMERA

        known = self._get_known_cameras()
        if camera_name not in known:
            result = ScanWorkspace.Result()
            result.success = False
            result.message = f"Unknown camera '{camera_name}'. Known: {known}"
            goal_handle.abort()
            return result

        with self._lock:
            self._scanning = True
        try:
            return self._run_scan(goal_handle, goal, camera_name)
        finally:
            with self._lock:
                self._scanning = False

    # -----------------------------------------------------------------------
    # Core scan logic
    # -----------------------------------------------------------------------

    def _run_scan(self, goal_handle, goal, camera_name):
        request_id = goal.request_id or ""
        workspace_id = goal.workspace_id or "workspace"
        duration_sec = float(goal.duration_sec)
        profile = CAMERA_PROFILES.get(camera_name, {})
        frame_id = profile.get("frame_id", "world")

        map_id = _make_map_id(workspace_id, camera_name, request_id)
        map_dir = os.path.join(
            self._get_map_save_dir(), workspace_id, camera_name, map_id
        )
        os.makedirs(map_dir, exist_ok=True)

        fb = ScanWorkspace.Feedback()

        def publish(progress, phase, message):
            fb.progress = float(progress)
            fb.phase = phase
            fb.message = message
            goal_handle.publish_feedback(fb)

        publish(0.0, "initializing", f"Loading scan config: {self._config_path}")

        poses = self._load_poses()
        if not poses:
            goal_handle.abort()
            result = ScanWorkspace.Result()
            result.success = False
            result.message = f"No poses found in {self._config_path}"
            return result

        total = len(poses)
        per_pose_wait = max(
            self.get_parameter("per_pose_wait_sec").get_parameter_value().double_value,
            1.0,
        )
        if duration_sec > 0:
            per_pose_wait = max(duration_sec / total, 1.0)

        publish(0.02, "initializing", f"Starting scan: {total} poses, {per_pose_wait:.1f}s/pose")

        self._xarm.set_planning_pipeline("ompl")

        for i, joint_values in enumerate(poses):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ScanWorkspace.Result()
                result.success = False
                result.message = "Scan canceled."
                return result

            progress = 0.05 + (i / total) * 0.75
            publish(progress, "moving", f"Moving to pose {i + 1}/{total}")

            joints = [float(v) for v in joint_values]
            self._xarm.set_joint_value_target(joints)
            success, _, move_dur, error_code = self._xarm.plan()

            if not success:
                self.get_logger().warn(
                    f"Planning failed at pose {i + 1}/{total}: error_code={error_code}"
                )
                publish(progress, "planning_failed", f"Pose {i + 1} planning failed, skipping")
                continue

            self.get_logger().info(f"Executing pose {i + 1}/{total} (plan_dur={move_dur:.2f}s)")
            if not self._xarm.execute():
                self.get_logger().warn(f"Execution failed at pose {i + 1}/{total}")
                publish(progress, "execution_failed", f"Pose {i + 1} execution failed, skipping")
                continue

            self._octomap_event.clear()
            got = self._octomap_event.wait(timeout=per_pose_wait)
            if not got:
                self.get_logger().warn(f"OctoMap timeout after pose {i + 1}/{total}")

            progress_after = 0.05 + ((i + 1) / total) * 0.75
            publish(progress_after, "scanning", f"Captured pose {i + 1}/{total}")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = ScanWorkspace.Result()
            result.success = False
            result.message = "Scan canceled after arm movement."
            return result

        publish(0.82, "saving", "Capturing final OctoMap...")

        octomap_msg = self._latest_octomap
        if octomap_msg is None:
            self._octomap_event.clear()
            self._octomap_event.wait(timeout=10.0)
            octomap_msg = self._latest_octomap

        # Statistics
        occupied_voxels = 0
        nearest_dist = -1.0
        cloud = self._latest_cloud
        if cloud is not None:
            occupied_voxels, nearest_dist = _stats_from_cloud(cloud)

        obstacle_detected = occupied_voxels > 0
        resolution = octomap_msg.resolution if octomap_msg else 0.1
        voxel_vol = resolution ** 3
        total_voxels = occupied_voxels
        occupancy_ratio = occupied_voxels / max(total_voxels, 1) if total_voxels > 0 else 0.0

        # Save octomap.bt
        octomap_path = os.path.join(map_dir, "octomap.bt")
        if octomap_msg is not None:
            _save_octomap_bt(octomap_msg, octomap_path)
            self.get_logger().info(
                f"Saved octomap.bt ({len(octomap_msg.data)} bytes) to {octomap_path}"
            )
        else:
            open(octomap_path, "wb").close()
            self.get_logger().warn("No OctoMap received; saved empty placeholder.")

        summary_data = {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "workspace_id": workspace_id,
            "camera_name": camera_name,
            "map_id": map_id,
            "map_path": map_dir,
            "frame_id": frame_id,
            "occupied_voxels": occupied_voxels,
            "free_voxels": 0,
            "unknown_voxels": 0,
            "occupancy_ratio": float(occupancy_ratio),
            "obstacle_detected": bool(obstacle_detected),
            "nearest_obstacle_distance_m": float(nearest_dist),
            "warnings": (["OctoMap not received"] if octomap_msg is None else []),
            "notes": [f"resolution={resolution}m", f"voxel_vol={voxel_vol:.6f}m3"],
        }
        metadata = {
            "request_id": request_id,
            "workspace_id": workspace_id,
            "camera_name": camera_name,
            "map_id": map_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "duration_sec": duration_sec,
            "frame_id": frame_id,
            "octomap_path": octomap_path,
            "summary_path": os.path.join(map_dir, "summary.json"),
            "num_poses": total,
            "per_pose_wait_sec": per_pose_wait,
        }

        with open(os.path.join(map_dir, "summary.json"), "w") as f:
            json.dump(summary_data, f, indent=2)
        with open(os.path.join(map_dir, "metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)

        # --- Register obstacles to planning scene (ObstacleRegistrar) ---
        publish(0.92, "registering", "Registering obstacles to MoveIt planning scene...")
        if octomap_msg is not None:
            self._registrar.apply(octomap_msg, clear_existing=False)
            self.get_logger().info("Obstacles registered to planning scene.")
        else:
            self.get_logger().warn("No OctoMap to register.")

        # Build ScanWorkspaceSummary message
        msg = ScanWorkspaceSummary()
        msg.schema_version = summary_data["schema_version"]
        msg.request_id = summary_data["request_id"]
        msg.workspace_id = summary_data["workspace_id"]
        msg.camera_name = summary_data["camera_name"]
        msg.map_id = summary_data["map_id"]
        msg.map_path = summary_data["map_path"]
        msg.frame_id = summary_data["frame_id"]
        msg.occupied_voxels = summary_data["occupied_voxels"]
        msg.free_voxels = summary_data["free_voxels"]
        msg.unknown_voxels = summary_data["unknown_voxels"]
        msg.occupancy_ratio = summary_data["occupancy_ratio"]
        msg.obstacle_detected = summary_data["obstacle_detected"]
        msg.nearest_obstacle_distance_m = summary_data["nearest_obstacle_distance_m"]
        msg.warnings = summary_data["warnings"]
        msg.notes = summary_data["notes"]

        publish(1.0, "done", f"Scan complete. map_id={map_id}")
        self.get_logger().info(f"Scan complete: {map_dir}")

        goal_handle.succeed()
        result = ScanWorkspace.Result()
        result.success = True
        result.message = f"Scan complete. {occupied_voxels} occupied voxels. Saved to {map_dir}"
        result.map_id = map_id
        result.map_path = map_dir
        result.summary_json = json.dumps(summary_data)
        result.summary = msg
        return result


def main(args=None):
    xarm_node = XArmNode("scan_workspace_xarm")
    rclpy.init(args=args)
    node = ScanWorkspaceActionServer(xarm_node)
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
