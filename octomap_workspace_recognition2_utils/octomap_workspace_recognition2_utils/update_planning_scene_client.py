import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from octomap_workspace_recognition2_interfaces.action import UpdatePlanningScene


class UpdatePlanningSceneClient(Node):
    def __init__(self):
        super().__init__("update_planning_scene_client_" + uuid.uuid4().hex[:6])
        self._client = ActionClient(self, UpdatePlanningScene, "update_planning_scene")

    def call(
        self,
        map_id,
        clear_existing=True,
        request_id=None,
        timeout_sec=60.0,
    ):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("update_planning_scene action server not available.")

        goal = UpdatePlanningScene.Goal()
        goal.request_id = request_id or uuid.uuid4().hex[:8]
        goal.map_id = map_id
        goal.clear_existing = clear_existing

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("update_planning_scene goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result().result

        return {
            "success": result.success,
            "message": result.message,
            "applied_objects": result.applied_objects,
            "applied_octomap": result.applied_octomap,
        }
