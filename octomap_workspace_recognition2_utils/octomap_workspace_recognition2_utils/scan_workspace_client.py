import json
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from octomap_workspace_recognition2_interfaces.action import ScanWorkspace


class ScanWorkspaceClient(Node):
    def __init__(self):
        super().__init__("scan_workspace_client_" + uuid.uuid4().hex[:6])
        self._client = ActionClient(self, ScanWorkspace, "scan_workspace")

    def call(
        self,
        workspace_id,
        duration_sec,
        camera_name="hand_camera",
        request_id=None,
        timeout_sec=120.0,
    ):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("scan_workspace action server not available.")

        goal = ScanWorkspace.Goal()
        goal.request_id = request_id or uuid.uuid4().hex[:8]
        goal.workspace_id = workspace_id
        goal.camera_name = camera_name
        goal.duration_sec = float(duration_sec)

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("scan_workspace goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result().result

        summary = {}
        if result.summary_json:
            try:
                summary = json.loads(result.summary_json)
            except json.JSONDecodeError:
                pass

        return {
            "success": result.success,
            "message": result.message,
            "map_id": result.map_id,
            "map_path": result.map_path,
            "summary_json": result.summary_json,
            "summary": summary,
        }
