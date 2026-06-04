"""Entry points for rag_factory_specific_task_agent."""
import rclpy

from octomap_workspace_recognition2_utils.scan_workspace_client import (
    ScanWorkspaceClient,
)
from octomap_workspace_recognition2_utils.update_planning_scene_client import (
    UpdatePlanningSceneClient,
)
from octomap_workspace_recognition2_utils.register_obstacles_client import (
    RegisterObstaclesClient,
)
from octomap_workspace_recognition2_utils.move_and_scan_obstacle_client import (
    MoveAndScanObstacleClient,
)


def scan_workspace(workspace_id, duration_sec, camera_name="hand_camera"):
    """Scan a workspace and return the result as a dict.

    Returns a dict with keys: success, message, map_id, map_path, summary_json, summary.
    """
    init_needed = not rclpy.ok()
    if init_needed:
        rclpy.init()
    node = ScanWorkspaceClient()
    try:
        return node.call(
            workspace_id=workspace_id,
            duration_sec=duration_sec,
            camera_name=camera_name,
        )
    finally:
        node.destroy_node()
        if init_needed and rclpy.ok():
            rclpy.shutdown()


def update_planning_scene(map_id, clear_existing=True):
    """Apply a saved map to the MoveIt PlanningScene.

    Returns a dict with keys: success, message, applied_objects, applied_octomap.
    """
    init_needed = not rclpy.ok()
    if init_needed:
        rclpy.init()
    node = UpdatePlanningSceneClient()
    try:
        return node.call(map_id=map_id, clear_existing=clear_existing)
    finally:
        node.destroy_node()
        if init_needed and rclpy.ok():
            rclpy.shutdown()


def register_obstacles(
    camera_name="hand_camera",
    clear_existing=False,
    capture_duration_sec=1.0,
):
    """Register the current camera view as obstacles in the MoveIt planning scene.

    The robot does not move. Captures the current OctoMap snapshot and applies it.
    Returns a dict with keys: success, message, occupied_voxels, map_id.
    """
    init_needed = not rclpy.ok()
    if init_needed:
        rclpy.init()
    node = RegisterObstaclesClient()
    try:
        return node.call(
            camera_name=camera_name,
            clear_existing=clear_existing,
            capture_duration_sec=float(capture_duration_sec),
        )
    finally:
        node.destroy_node()
        if init_needed and rclpy.ok():
            rclpy.shutdown()


def move_and_scan_obstacle(
    target_joints=None,
    target_pose=None,
    target_pose_frame_id="link_base",
    camera_name="hand_camera",
    stability_wait_sec=2.0,
    joint_velocity_threshold=0.01,
    clear_existing=False,
):
    """Move robot to target joints or pose, wait for stability, scan, and register obstacles.

    Specify either target_joints (list of 6 float [rad]) or target_pose
    (dict with keys "position" and "orientation").
    Returns a dict with keys: success, message, final_joints, occupied_voxels, map_id.
    """
    init_needed = not rclpy.ok()
    if init_needed:
        rclpy.init()
    node = MoveAndScanObstacleClient()
    try:
        return node.call(
            target_joints=target_joints,
            target_pose=target_pose,
            target_pose_frame_id=target_pose_frame_id,
            camera_name=camera_name,
            stability_wait_sec=float(stability_wait_sec),
            joint_velocity_threshold=float(joint_velocity_threshold),
            clear_existing=clear_existing,
        )
    finally:
        node.destroy_node()
        if init_needed and rclpy.ok():
            rclpy.shutdown()
