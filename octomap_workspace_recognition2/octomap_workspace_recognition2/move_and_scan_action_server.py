import threading
import time
import uuid
from datetime import datetime

import rclpy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPositionIK
from octomap_msgs.msg import Octomap
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from xarm_utils_py import XArmUtils
from xarm_utils_py import Node as XArmNode
from geometry_msgs.msg import PoseStamped

from octomap_workspace_recognition2.mapping_move import default_config_path
from octomap_workspace_recognition2.obstacle_registrar import ObstacleRegistrar
from octomap_workspace_recognition2_interfaces.action import MoveAndScanObstacle

DEFAULT_CAMERA = "hand_camera"
DEFAULT_OCTOMAP_TOPIC = "/octomap_static/octomap_binary"
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"


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
    return f"move_scan_{safe_cam}_{ts}_{suffix}"


class MoveAndScanActionServer(LifecycleNode):
    """
    指定 joints または pose にロボットを移動させ、安定後に OctoMap をスキャンして
    MoveIt planning scene に障害物登録する。
    登録は ObstacleRegistrar.apply() を通じて行い、RegisterObstacles と同一ロジックを共有する。
    """

    def __init__(self, xarm_node: XArmNode):
        super().__init__("move_and_scan_action_server")
        self._xarm_node = xarm_node

    # -----------------------------------------------------------------------
    # Lifecycle transitions
    # -----------------------------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.declare_parameter("move_group", "xarm6")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("octomap_topic", DEFAULT_OCTOMAP_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)

        move_group = (
            self.get_parameter("move_group").get_parameter_value().string_value or "xarm6"
        )
        self._xarm = XArmUtils(self._xarm_node, move_group)

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
        self._octomap_event = threading.Event()
        self._latest_joint_velocities: list = []
        self._lock = threading.Lock()
        self._busy = False
        self._is_active = False
        self._octomap_sub = None
        self._joint_states_sub = None

        # IK service client (used when use_joints=false)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # Action server created once; destroyed in on_cleanup only.
        action_cg = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveAndScanObstacle,
            "move_and_scan_obstacle",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_cg,
        )
        self.get_logger().info("MoveAndScan: configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        octomap_topic = (
            self.get_parameter("octomap_topic").get_parameter_value().string_value
            or DEFAULT_OCTOMAP_TOPIC
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
            or DEFAULT_JOINT_STATES_TOPIC
        )

        sub_cg1 = MutuallyExclusiveCallbackGroup()
        self._octomap_sub = self.create_subscription(
            Octomap,
            octomap_topic,
            self._on_octomap,
            _sensor_qos(),
            callback_group=sub_cg1,
        )
        sub_cg2 = MutuallyExclusiveCallbackGroup()
        self._joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            _sensor_qos(),
            callback_group=sub_cg2,
        )
        self._is_active = True
        self.get_logger().info("MoveAndScan action server active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._is_active = False
        if self._octomap_sub is not None:
            self.destroy_subscription(self._octomap_sub)
            self._octomap_sub = None
        if self._joint_states_sub is not None:
            self.destroy_subscription(self._joint_states_sub)
            self._joint_states_sub = None
        self.get_logger().info("MoveAndScan: deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._scene_pub)
        self.get_logger().info("MoveAndScan: cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # -----------------------------------------------------------------------
    # Sensor callbacks
    # -----------------------------------------------------------------------

    def _on_octomap(self, msg: Octomap):
        self._latest_octomap = msg
        self._octomap_event.set()

    def _on_joint_states(self, msg: JointState):
        self._latest_joint_velocities = list(msg.velocity)

    # -----------------------------------------------------------------------
    # Action callbacks
    # -----------------------------------------------------------------------

    def _goal_callback(self, _):
        if not self._is_active:
            self.get_logger().warn("MoveAndScan: not active, rejecting goal.")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn("MoveAndScan: rejecting goal, already busy.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._lock:
            self._busy = True
        try:
            return self._run(goal_handle)
        finally:
            with self._lock:
                self._busy = False

    # -----------------------------------------------------------------------
    # Core logic
    # -----------------------------------------------------------------------

    def _run(self, goal_handle):
        goal = goal_handle.request
        request_id = goal.request_id or ""
        camera_name = (goal.camera_name or "").strip() or DEFAULT_CAMERA
        stability_wait_sec = max(float(goal.stability_wait_sec or 2.0), 0.0)
        vel_threshold = float(goal.joint_velocity_threshold or 0.0)
        clear_existing = bool(goal.clear_existing)

        fb = MoveAndScanObstacle.Feedback()

        def publish_fb(progress, phase, message):
            fb.progress = float(progress)
            fb.phase = phase
            fb.message = message
            goal_handle.publish_feedback(fb)

        # --- 1. Determine target joints ---
        if goal.use_joints:
            target_joints = list(goal.target_joints)
            if not target_joints:
                goal_handle.abort()
                result = MoveAndScanObstacle.Result()
                result.success = False
                result.message = "use_joints=true but target_joints is empty."
                return result
        else:
            publish_fb(0.05, "planning", "Computing IK for target pose...")
            target_joints = self._compute_ik(goal)
            if target_joints is None:
                goal_handle.abort()
                result = MoveAndScanObstacle.Result()
                result.success = False
                result.message = "IK computation failed."
                return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = MoveAndScanObstacle.Result()
            result.success = False
            result.message = "Canceled before movement."
            return result

        # --- 2. Plan and execute ---
        publish_fb(0.1, "planning", f"Planning to target joints: {target_joints}")
        self._xarm.set_planning_pipeline("ompl")
        self._xarm.set_joint_value_target(target_joints)
        success, _, move_dur, error_code = self._xarm.plan()

        if not success:
            goal_handle.abort()
            result = MoveAndScanObstacle.Result()
            result.success = False
            result.message = f"Motion planning failed: error_code={error_code}"
            return result

        publish_fb(0.2, "moving", f"Executing trajectory (planned {move_dur:.2f}s)...")
        if not self._xarm.execute():
            goal_handle.abort()
            result = MoveAndScanObstacle.Result()
            result.success = False
            result.message = "Trajectory execution failed."
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = MoveAndScanObstacle.Result()
            result.success = False
            result.message = "Canceled after movement."
            return result

        # --- 3. Wait for stability ---
        publish_fb(0.5, "waiting_stability", f"Waiting {stability_wait_sec:.1f}s for arm to stabilize...")
        deadline = time.time() + stability_wait_sec
        while time.time() < deadline:
            time.sleep(0.05)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveAndScanObstacle.Result()
                result.success = False
                result.message = "Canceled during stability wait."
                return result

        if vel_threshold > 0.0 and self._latest_joint_velocities:
            max_vel_timeout = time.time() + 5.0
            while time.time() < max_vel_timeout:
                vels = self._latest_joint_velocities
                if vels and all(abs(v) < vel_threshold for v in vels):
                    self.get_logger().info("Arm stabilized (velocity threshold met).")
                    break
                time.sleep(0.05)

        # --- 4. Capture and register OctoMap (via ObstacleRegistrar) ---
        publish_fb(0.7, "scanning", "Waiting for OctoMap snapshot...")
        self._octomap_event.clear()
        got = self._octomap_event.wait(timeout=5.0)

        if not got and self._latest_octomap is None:
            goal_handle.abort()
            result = MoveAndScanObstacle.Result()
            result.success = False
            result.message = "No OctoMap received after movement."
            return result

        publish_fb(0.9, "registering", "Registering obstacles to planning scene...")
        ok = self._registrar.apply(self._latest_octomap, clear_existing=clear_existing)

        map_id = _make_map_id(camera_name, request_id)
        publish_fb(1.0, "done", f"Done. map_id={map_id}")
        goal_handle.succeed()

        result = MoveAndScanObstacle.Result()
        result.success = ok
        result.message = "Move, scan, and register complete." if ok else "Registration failed."
        result.final_joints = target_joints
        result.occupied_voxels = 0
        result.map_id = map_id
        return result

    def _compute_ik(self, goal) -> list | None:
        """target_pose から /compute_ik で joint angles を求める。"""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service /compute_ik not available.")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = (
            self.get_parameter("move_group").get_parameter_value().string_value or "xarm6"
        )
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = (
            goal.target_pose_frame_id or "link_base"
        )
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = goal.target_pose
        req.ik_request.timeout.sec = 5

        future = self._ik_client.call_async(req)
        # spin_until_future_complete は executor コンテキスト外なので短い手動ポーリングで代替
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self.get_logger().error("IK service call timed out or failed.")
            return None

        resp = future.result()
        if resp.error_code.val != resp.error_code.SUCCESS:
            self.get_logger().error(f"IK failed: error_code={resp.error_code.val}")
            return None

        return list(resp.solution.joint_state.position)


def main(args=None):
    xarm_node = XArmNode("move_and_scan_xarm")
    rclpy.init(args=args)
    node = MoveAndScanActionServer(xarm_node)
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
