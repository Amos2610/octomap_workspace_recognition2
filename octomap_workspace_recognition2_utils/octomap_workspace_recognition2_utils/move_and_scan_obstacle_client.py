import uuid

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node

from octomap_workspace_recognition2_interfaces.action import MoveAndScanObstacle


class MoveAndScanObstacleClient(Node):
    def __init__(self):
        super().__init__("move_and_scan_obstacle_client_" + uuid.uuid4().hex[:6])
        self._client = ActionClient(self, MoveAndScanObstacle, "move_and_scan_obstacle")

    def call(
        self,
        target_joints: list = None,
        target_pose: dict = None,
        target_pose_frame_id: str = "link_base",
        camera_name: str = "hand_camera",
        stability_wait_sec: float = 2.0,
        joint_velocity_threshold: float = 0.01,
        clear_existing: bool = False,
        request_id: str = None,
        timeout_sec: float = 120.0,
    ) -> dict:
        """
        target_joints または target_pose のいずれか一方を指定する。
        target_pose は {"position": {"x": ..., "y": ..., "z": ...},
                         "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}} の形式。
        """
        if target_joints is None and target_pose is None:
            raise ValueError("Either target_joints or target_pose must be specified.")

        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("move_and_scan_obstacle action server not available.")

        goal = MoveAndScanObstacle.Goal()
        goal.request_id = request_id or uuid.uuid4().hex[:8]
        goal.camera_name = camera_name
        goal.stability_wait_sec = float(stability_wait_sec)
        goal.joint_velocity_threshold = float(joint_velocity_threshold)
        goal.clear_existing = clear_existing

        if target_joints is not None:
            goal.use_joints = True
            goal.target_joints = [float(j) for j in target_joints]
        else:
            goal.use_joints = False
            goal.target_pose_frame_id = target_pose_frame_id
            pose_msg = Pose()
            pos = target_pose.get("position", {})
            ori = target_pose.get("orientation", {})
            pose_msg.position.x = float(pos.get("x", 0.0))
            pose_msg.position.y = float(pos.get("y", 0.0))
            pose_msg.position.z = float(pos.get("z", 0.0))
            pose_msg.orientation.x = float(ori.get("x", 0.0))
            pose_msg.orientation.y = float(ori.get("y", 0.0))
            pose_msg.orientation.z = float(ori.get("z", 0.0))
            pose_msg.orientation.w = float(ori.get("w", 1.0))
            goal.target_pose = pose_msg

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("move_and_scan_obstacle goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result().result

        return {
            "success": result.success,
            "message": result.message,
            "final_joints": list(result.final_joints),
            "occupied_voxels": result.occupied_voxels,
            "map_id": result.map_id,
        }
