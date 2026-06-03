import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from octomap_workspace_recognition2_interfaces.action import RegisterObstacles


class RegisterObstaclesClient(Node):
    def __init__(self):
        super().__init__("register_obstacles_client_" + uuid.uuid4().hex[:6])
        self._client = ActionClient(self, RegisterObstacles, "register_obstacles")

    def call(
        self,
        camera_name: str = "hand_camera",
        clear_existing: bool = False,
        capture_duration_sec: float = 1.0,
        request_id: str = None,
        timeout_sec: float = 30.0,
    ) -> dict:
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("register_obstacles action server not available.")

        goal = RegisterObstacles.Goal()
        goal.request_id = request_id or uuid.uuid4().hex[:8]
        goal.camera_name = camera_name
        goal.clear_existing = clear_existing
        goal.capture_duration_sec = float(capture_duration_sec)

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("register_obstacles goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result().result

        return {
            "success": result.success,
            "message": result.message,
            "occupied_voxels": result.occupied_voxels,
            "map_id": result.map_id,
        }
