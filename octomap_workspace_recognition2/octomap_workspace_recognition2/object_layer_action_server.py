"""Object layer: 掴んだ物体を planning scene で手先に attach / detach する。

AttachObject と DetachObject の action server。ロボットは動かさない。

形状は呼び出し側が渡す（この package は検出器に依存しない）。goal は mesh か
primitive を持ち、mesh に頂点があれば mesh が優先、無ければ primitive を使う。

planning scene への書き込みは ``/apply_planning_scene`` サービスで行う
（``/planning_scene`` topic は fire-and-forget で成否が取れない）。
"""

import re
import threading

import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from octomap_workspace_recognition2_interfaces.action import AttachObject, DetachObject

NODE_NAME = "object_layer_action_server"

#: goal の attach_link が空のときの既定。xArm6 の MoveIt 構成では link_tcp が
#: end effector の親リンク（SRDF の end_effector parent_link）。
DEFAULT_ATTACH_LINK = "link_tcp"

#: 凸包なら 30 頂点前後。これを超えるものは間引かれていない点群であり、
#: planning scene に載せると以後の全衝突検査が遅くなる。
MAX_MESH_VERTICES = 2000

#: これ未満では何も囲めないので「mesh 無し」と読む（壊れた mesh とは区別する）。
MIN_MESH_VERTICES = 4


def call_sync(client, request, timeout_sec: float):
    """action の execute コールバック内からサービスを呼び、応答を待つ。

    ``spin_until_future_complete`` は使わない。executor が既にこのノードを別スレッドで
    回しているので、ここで再度 spin すると入れ子 spin のデッドロックになる。
    future の done コールバックで Event を立てて待てば、応答は executor のスレッドで
    配送される。timeout なら None。
    """
    future = client.call_async(request)
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_sec):
        future.cancel()
        return None
    return future.result()


def mesh_problem(mesh) -> str:
    """この mesh が使えない理由。使えるなら空文字。

    **空の mesh は問題なし**（無いのと壊れているのは別。呼び出し側は primitive に
    落ちる）。内容があって壊れているものだけ理由を返し、goal は拒否する。
    壊れた mesh を黙って箱に降格すると、呼び出し側の不具合が planning scene 上では
    もっともらしい形として隠れてしまう。
    """
    vertices, triangles = mesh.vertices, mesh.triangles
    if not vertices and not triangles:
        return ""
    if len(vertices) < MIN_MESH_VERTICES:
        return f"mesh has {len(vertices)} vertices; {MIN_MESH_VERTICES} are needed to enclose anything"
    if len(vertices) > MAX_MESH_VERTICES:
        return f"mesh has {len(vertices)} vertices, over the {MAX_MESH_VERTICES} limit"
    if not triangles:
        return f"mesh has {len(vertices)} vertices but no triangles"
    for vertex in vertices:
        for value in (vertex.x, vertex.y, vertex.z):
            if value != value or value in (float("inf"), float("-inf")):
                return "mesh has a non-finite vertex"
    for triangle in triangles:
        for index in triangle.vertex_indices:
            if index >= len(vertices):
                return (
                    f"mesh triangle refers to vertex {index}, "
                    f"past the end of its {len(vertices)} vertices"
                )
    return ""


class SceneWriter:
    """AttachedCollisionObject / CollisionObject の差分を move_group に適用する。"""

    def __init__(self, node, apply_service: str, callback_group=None):
        self._node = node
        self._apply_service = apply_service
        self._apply = node.create_client(ApplyPlanningScene, apply_service, callback_group=callback_group)

    def _apply_scene(self, scene: PlanningScene, timeout_sec: float) -> bool:
        if not self._apply.service_is_ready() and not self._apply.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(f"{self._apply_service} is not available; is move_group running?")
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = call_sync(self._apply, request, timeout_sec)
        if response is None:
            self._node.get_logger().warn(f"{self._apply_service} did not answer within {timeout_sec:.1f}s")
            return False
        return bool(response.success)

    def attach(self, collision_object, link_name: str, timeout_sec: float, touch_links=None) -> bool:
        """物体をリンクに attach し、プランナが腕と一緒に運ぶ扱いにする。

        world に先に ADD してから attach する 2 往復にはしない。その間に「グリッパの
        位置にある world 障害物」の窓ができ、その間の計画が失敗する。attached body の
        中に物体を入れて 1 往復で渡す。

        ``touch_links`` は物体に触れてよいリンク。運ぶリンクだけでは足りない
        （手先に溶接された隣のリンクとは姿勢によらず接触する）ので、呼び出し側が
        剛体クラスタごと渡す。
        """
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object = collision_object
        attached.object.operation = CollisionObject.ADD
        allowed = list(touch_links) if touch_links else []
        if link_name not in allowed:
            allowed.append(link_name)
        attached.touch_links = allowed

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply_scene(scene, timeout_sec)

    def detach(self, object_id: str, link_name: str, remove: bool, timeout_sec: float) -> bool:
        """detach する。remove が偽なら MoveIt の仕様どおり物体は world に戻る。"""
        detached = AttachedCollisionObject()
        detached.link_name = link_name
        detached.object.id = object_id
        detached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detached]
        if not self._apply_scene(scene, timeout_sec):
            return False
        if not remove:
            return True

        gone = CollisionObject()
        gone.id = object_id
        gone.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [gone]
        return self._apply_scene(scene, timeout_sec)


class ObjectLayerActionServer(LifecycleNode):
    def __init__(self, **kwargs) -> None:
        super().__init__(NODE_NAME, **kwargs)
        self._lock = threading.Lock()
        self._is_active = False
        # object_id -> attach したリンク。detach に要る。無い id の detach は
        # 「呼び出し側とプランナで持ち物の認識が食い違っている」状態なので失敗にする
        self._attached: dict = {}
        self._link_names: set = set()
        self._fixed_pairs: list = []

    # ------------------------------------------------------------------ lifecycle
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.declare_parameter("default_attach_link", DEFAULT_ATTACH_LINK)
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("service_timeout_sec", 10.0)
        # URDF の fixed joint で手先に溶接されたリンクに加えて、物体に触れてよいリンク。
        # 可動関節の先（グリッパの指など）は自動では含まれないので、指で掴むハンドは
        # ここに指のリンクを列挙する。空文字は無視する
        self.declare_parameter("extra_touch_links", [""])

        self._action_group = ReentrantCallbackGroup()
        self._client_group = MutuallyExclusiveCallbackGroup()
        self._scene = SceneWriter(
            self,
            apply_service=self.get_parameter("apply_planning_scene_service").value,
            callback_group=self._client_group,
        )
        # robot_state_publisher が latch している URDF。モデルに無いリンクへの attach は
        # サービスが受理した上で何もしないので、リンク名を知って先に断る
        self.create_subscription(
            String,
            self.get_parameter("robot_description_topic").value,
            self._on_robot_description,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
            callback_group=self._client_group,
        )
        self._attach_server = self._make_server(AttachObject, "attach_object", self._execute_attach)
        self._detach_server = self._make_server(DetachObject, "detach_object", self._execute_detach)
        self.get_logger().info(f"{NODE_NAME}: configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info(f"{NODE_NAME}: active")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ plumbing
    def _make_server(self, action_type, name: str, execute_cb) -> ActionServer:
        return ActionServer(
            self, action_type, name,
            execute_callback=execute_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if not self._is_active:
            self.get_logger().warn("goal refused: node is not active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(goal_handle, feedback_msg, progress: float, phase: str, message: str) -> None:
        feedback_msg.progress = float(progress)
        feedback_msg.phase = phase
        feedback_msg.message = message
        goal_handle.publish_feedback(feedback_msg)

    def _on_robot_description(self, msg: String) -> None:
        self._link_names = set(re.findall(r'<link\s+name="([^"]+)"', msg.data))
        self._fixed_pairs = self._parse_fixed_joints(msg.data)
        self.get_logger().info(
            f"robot model has {len(self._link_names)} links and {len(self._fixed_pairs)} fixed joints"
        )

    @staticmethod
    def _parse_fixed_joints(urdf: str):
        """URDF の fixed joint の (parent, child)。可動関節の先は本当に物体と衝突しうるので含めない。"""
        pairs = []
        for match in re.finditer(r"<joint\b([^>]*)>(.*?)</joint>", urdf, re.S):
            attrs, body = match.group(1), match.group(2)
            if 'type="fixed"' not in attrs:
                continue
            parent = re.search(r'<parent\s+link="([^"]+)"', body)
            child = re.search(r'<child\s+link="([^"]+)"', body)
            if parent and child:
                pairs.append((parent.group(1), child.group(1)))
        return pairs

    def _rigid_cluster(self, link: str):
        """``link`` に fixed joint で溶接されたリンク群（URDF 未受信なら link 自身だけ）。"""
        neighbours = {}
        for parent, child in self._fixed_pairs:
            neighbours.setdefault(parent, set()).add(child)
            neighbours.setdefault(child, set()).add(parent)
        seen = {link}
        queue = [link]
        while queue:
            current = queue.pop()
            for nxt in neighbours.get(current, ()):
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return sorted(seen)

    def _touch_links(self, link: str):
        allowed = set(self._rigid_cluster(link))
        extra = self.get_parameter("extra_touch_links").value or []
        allowed.update(name for name in extra if name)
        return sorted(allowed)

    @staticmethod
    def _fail(goal_handle, action_type, message: str, phase: str):
        feedback = action_type.Feedback()
        feedback.progress = 1.0
        feedback.phase = phase
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        result = action_type.Result()
        result.success = False
        result.message = message
        goal_handle.abort()
        return result

    def _timeout(self) -> float:
        return float(self.get_parameter("service_timeout_sec").value)

    def _resolve_link(self, requested: str) -> str:
        return (requested or "").strip() or self.get_parameter("default_attach_link").value

    # ------------------------------------------------------------------ executions
    def _execute_attach(self, goal_handle):
        goal = goal_handle.request
        feedback = AttachObject.Feedback()

        if not goal.object_id:
            return self._fail(goal_handle, AttachObject, "object_id must not be empty", "bad_request")

        link = self._resolve_link(goal.attach_link)
        if self._link_names and link not in self._link_names:
            return self._fail(
                goal_handle, AttachObject,
                f"the robot model has no link {link!r}; known links: {sorted(self._link_names)}",
                "unknown_link",
            )

        problem = mesh_problem(goal.mesh)
        if problem:
            return self._fail(goal_handle, AttachObject, problem, "bad_mesh")

        has_mesh = len(goal.mesh.vertices) >= MIN_MESH_VERTICES and bool(goal.mesh.triangles)
        if not has_mesh and not goal.primitive.dimensions:
            return self._fail(
                goal_handle, AttachObject,
                "neither a mesh nor a primitive was given; the caller supplies the shape",
                "bad_request",
            )

        frame_id = (goal.frame_id or "").strip() or self.get_parameter("planning_frame").value

        obj = CollisionObject()
        obj.id = goal.object_id
        obj.header.frame_id = frame_id
        if has_mesh:
            obj.meshes = [goal.mesh]
            obj.mesh_poses = [goal.pose]
            shape_desc = f"mesh({len(goal.mesh.vertices)}v/{len(goal.mesh.triangles)}t)"
        else:
            obj.primitives = [goal.primitive]
            obj.primitive_poses = [goal.pose]
            shape_desc = f"box{[round(d, 4) for d in goal.primitive.dimensions]}"

        touch_links = self._touch_links(link)
        self._feedback(goal_handle, feedback, 0.3, "attaching", f"{goal.object_id} to {link} in {frame_id}")
        self.get_logger().info(f"attaching {goal.object_id} as {shape_desc} to {link}; touch_links={touch_links}")
        if not self._scene.attach(obj, link, self._timeout(), touch_links=touch_links):
            return self._fail(goal_handle, AttachObject, "move_group refused the attach", "attaching")

        with self._lock:
            self._attached[goal.object_id] = link

        self._feedback(goal_handle, feedback, 1.0, "done", "attached")
        result = AttachObject.Result()
        result.success = True
        result.message = f"{goal.object_id} is attached to {link}"
        goal_handle.succeed()
        return result

    def _execute_detach(self, goal_handle):
        goal = goal_handle.request
        feedback = DetachObject.Feedback()

        with self._lock:
            link = self._attached.get(goal.object_id)
        if link is None:
            return self._fail(
                goal_handle, DetachObject,
                f"{goal.object_id!r} was never attached by this server",
                "unknown_object",
            )

        self._feedback(goal_handle, feedback, 0.3, "detaching", f"{goal.object_id} from {link}")
        if not self._scene.detach(goal.object_id, link, goal.remove_from_scene, self._timeout()):
            return self._fail(goal_handle, DetachObject, "move_group refused the detach", "detaching")

        with self._lock:
            self._attached.pop(goal.object_id, None)

        where = "removed from the scene" if goal.remove_from_scene else "left in the scene"
        self._feedback(goal_handle, feedback, 1.0, "done", where)
        result = DetachObject.Result()
        result.success = True
        result.message = f"{goal.object_id} detached from {link} and {where}"
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectLayerActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # 遷移は executor が回り始めてから要求する（spin 前に inline で呼ぶと遷移が落ちる）
    def _auto_start() -> None:
        threading.Event().wait(0.5)
        node.trigger_configure()
        node.trigger_activate()
    threading.Thread(target=_auto_start, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
