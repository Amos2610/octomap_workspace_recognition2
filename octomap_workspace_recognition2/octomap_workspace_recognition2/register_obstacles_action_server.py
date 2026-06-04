import threading
import uuid
from datetime import datetime

import rclpy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from octomap_workspace_recognition2.obstacle_registrar import ObstacleRegistrar
from octomap_workspace_recognition2_interfaces.action import RegisterObstacles

DEFAULT_CAMERA = "hand_camera"
DEFAULT_OCTOMAP_TOPIC = "/octomap_static/octomap_binary"


def _sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _make_map_id(camera_name: str, request_id: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_cam = camera_name.replace(" ", "_").replace("/", "_")
    suffix = (request_id[-4:].lower() if request_id else uuid.uuid4().hex[:4])
    return f"obstacles_{safe_cam}_{ts}_{suffix}"


class RegisterObstaclesActionServer(LifecycleNode):
    """
    現在の OctoMap を即時 MoveIt planning scene に登録するコアアクション。
    ロボットは動かさない。ScanWorkspace / MoveAndScanObstacle も
    ObstacleRegistrar 経由で同じ登録ロジックを共有する。
    """

    def __init__(self):
        super().__init__("register_obstacles_action_server")

    # -----------------------------------------------------------------------
    # Lifecycle transitions
    # -----------------------------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter("known_cameras", DEFAULT_CAMERA)
        self.declare_parameter("octomap_topic", DEFAULT_OCTOMAP_TOPIC)
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("frame_id", "world")

        self._latest_octomap: Octomap = None
        self._octomap_event = threading.Event()
        self._lock = threading.Lock()
        self._busy = False
        self._is_active = False

        scene_topic = (
            self.get_parameter("planning_scene_topic").get_parameter_value().string_value
            or "/planning_scene"
        )
        frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value or "world"
        )
        self._scene_pub = self.create_publisher(PlanningScene, scene_topic, 10)
        self._registrar = ObstacleRegistrar(self, self._scene_pub, frame_id)

        # Action server is created once here and never destroyed while spinning.
        # Goal acceptance is controlled by _is_active flag instead.
        action_cg = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            RegisterObstacles,
            "register_obstacles",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_cg,
        )
        self._octomap_sub = None
        self.get_logger().info("RegisterObstacles: configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        octomap_topic = (
            self.get_parameter("octomap_topic").get_parameter_value().string_value
            or DEFAULT_OCTOMAP_TOPIC
        )
        sub_cg = MutuallyExclusiveCallbackGroup()
        self._octomap_sub = self.create_subscription(
            Octomap,
            octomap_topic,
            self._on_octomap,
            _sensor_qos(),
            callback_group=sub_cg,
        )
        self._is_active = True
        self.get_logger().info("RegisterObstacles action server active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        if self._octomap_sub is not None:
            self.destroy_subscription(self._octomap_sub)
            self._octomap_sub = None
        self.get_logger().info("RegisterObstacles: deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._scene_pub)
        self.get_logger().info("RegisterObstacles: cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _on_octomap(self, msg: Octomap):
        self._latest_octomap = msg
        self._octomap_event.set()

    def _goal_callback(self, _):
        if not self._is_active:
            self.get_logger().warn("RegisterObstacles: not active, rejecting goal.")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn("RegisterObstacles: already busy, rejecting goal.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        camera_name = (goal.camera_name or "").strip() or DEFAULT_CAMERA
        capture_duration_sec = max(float(goal.capture_duration_sec or 1.0), 0.1)
        clear_existing = bool(goal.clear_existing)
        request_id = goal.request_id or ""

        with self._lock:
            self._busy = True
        try:
            return self._run(goal_handle, camera_name, capture_duration_sec, clear_existing, request_id)
        finally:
            with self._lock:
                self._busy = False

    # -----------------------------------------------------------------------
    # Core logic
    # -----------------------------------------------------------------------

    def _run(self, goal_handle, camera_name, capture_duration_sec, clear_existing, request_id):
        fb = RegisterObstacles.Feedback()

        def publish_fb(progress, phase, message):
            fb.progress = float(progress)
            fb.phase = phase
            fb.message = message
            goal_handle.publish_feedback(fb)

        publish_fb(0.0, "waiting_octomap", "Waiting for OctoMap data...")

        self._octomap_event.clear()
        got = self._octomap_event.wait(timeout=capture_duration_sec + 2.0)

        if not got and self._latest_octomap is None:
            goal_handle.abort()
            result = RegisterObstacles.Result()
            result.success = False
            result.message = "No OctoMap received within timeout."
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = RegisterObstacles.Result()
            result.success = False
            result.message = "Canceled."
            return result

        octomap_msg = self._latest_octomap
        publish_fb(0.5, "processing", "OctoMap received, registering...")

        publish_fb(0.8, "registering", "Publishing to /planning_scene...")
        ok = self._registrar.apply(octomap_msg, clear_existing=clear_existing)

        map_id = _make_map_id(camera_name, request_id)
        publish_fb(1.0, "done", f"Done. map_id={map_id}")
        goal_handle.succeed()

        result = RegisterObstacles.Result()
        result.success = ok
        result.message = "Obstacles registered to planning scene." if ok else "Registration failed."
        result.occupied_voxels = 0
        result.map_id = map_id
        return result


def main(args=None):
    import threading
    rclpy.init(args=args)
    node = RegisterObstaclesActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _auto_start():
        import time
        time.sleep(0.5)
        node.trigger_configure()
        node.trigger_activate()

    threading.Thread(target=_auto_start, daemon=True).start()

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
