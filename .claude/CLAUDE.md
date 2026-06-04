# Coding AI Implementation Instructions

Target repository:

```text
ros2_ws/src/octomap_workspace_recognition2
```

## Purpose

Expose the existing OctoMap workspace recognition implementation as ROS 2 Actions without broadly reorganizing the current code.

`rag_factory_specific_task_agent` should call the functionality through `octomap_workspace_recognition2_utils`, which wraps:

- `ScanWorkspace` Action
- `UpdatePlanningScene` Action

## Important Policy

- Preserve the existing file structure.
- Do not create large new responsibility directories such as `core`, `scan`, or `query`.
- `scan_workspace_action_server.py` must be a thin Action wrapper around the existing recognition flow.
- `update_planning_scene_action_server.py` must be a thin Action wrapper around the existing MoveIt PlanningScene update flow.
- `rag_factory_specific_task_agent` must not import runtime modules from `octomap_workspace_recognition2` directly.
- `octomap_workspace_recognition2_interfaces` must contain interface definitions only. Do not put runtime implementation there.

## Packages To Implement

### `octomap_workspace_recognition2`

Existing runtime package.

Add or update:

- `scan_workspace_action_server.py`
- `update_planning_scene_action_server.py`
- launch file entries for both Action Servers
- map save directory parameter
- camera profile configuration

Use existing modules first:

```text
autonomous_recognition.py
mapping_move.py
convert_octomap.py
octomap_to_moveit.py
collision_to_moveit.py
yolo_detection.py
```

### `octomap_workspace_recognition2_interfaces`

Interface-only package.

Add or update:

- `action/ScanWorkspace.action`
- `action/UpdatePlanningScene.action`
- `msg/ScanWorkspaceSummary.msg`

Keep existing `OccupancySummary.msg`. Replace its use with `ScanWorkspaceSummary.msg` where appropriate.

### `octomap_workspace_recognition2_utils`

Agent-side utility package.

Add:

- `scan_workspace_client.py`
- `update_planning_scene_client.py`
- `rag_tool.py`

## `ScanWorkspace.action`

### Goal

```text
string request_id
string workspace_id
string camera_name
float64 duration_sec
```

### Result

```text
bool success
string message
string map_id
string map_path
string summary_json
octomap_workspace_recognition2_interfaces/ScanWorkspaceSummary summary
```

### Feedback

```text
float32 progress
string phase
string message
```

## Camera Name Handling

- If `camera_name` is empty, use `hand_camera`.
- If `camera_name` is unknown, do not start scanning. Return `success=false` with a clear message.
- `camera_name` is expected to support three cameras in the future.
- The first default is `hand_camera`.
- Camera topic, `frame_id`, and other required settings must be loadable from launch parameters or YAML config.
- Do not hard-code future camera names in implementation. Only `hand_camera` may be the default.

## Single Scan Execution

`ScanWorkspace` Action Server must accept only one scan goal at a time.

- If another scan goal arrives while scanning, reject it as busy without starting a scan.
- An in-process lock is enough for the first implementation.
- In the busy case, use phase `rejected_busy`.
- The busy message must clearly state that another scan is currently running.

## Map Save Specification

Scan results must always be saved to files.

The save base directory must be configurable through a launch parameter:

```text
map_save_dir
```

If `map_save_dir` is unspecified, default to:

```text
<package_share>/io/maps
```

Save directory structure:

```text
map_save_dir/workspace_id/camera_name/map_id/
```

Required files:

```text
octomap.bt
summary.json
metadata.json
```

Optional files:

```text
collision_objects.json
debug.json
```

`map_id` must be generated from:

- `workspace_id`
- `camera_name`
- timestamp
- request ID suffix

Example:

```text
front_workspace_hand_camera_20260601_143012_a7f9
```

`metadata.json` must include at least:

```text
request_id
workspace_id
camera_name
map_id
created_at
duration_sec
frame_id
octomap_path
summary_path
```

`summary_json` must include at least:

```text
schema_version
request_id
workspace_id
camera_name
map_id
map_path
frame_id
occupied_voxels
free_voxels
unknown_voxels
occupancy_ratio
obstacle_detected
nearest_obstacle_distance_m
warnings
notes
```

## `ScanWorkspaceSummary.msg`

Add this message:

```text
string schema_version
string request_id
string workspace_id
string camera_name
string map_id
string map_path
string frame_id
int32 occupied_voxels
int32 free_voxels
int32 unknown_voxels
float64 occupancy_ratio
bool obstacle_detected
float64 nearest_obstacle_distance_m
string[] warnings
string[] notes
```

`schema_version` must be:

```text
scan_workspace_summary.v1
```

## `UpdatePlanningScene.action`

### Goal

```text
string request_id
string map_id
bool clear_existing
```

### Result

```text
bool success
string message
int32 applied_objects
bool applied_octomap
```

### Feedback

```text
float32 progress
string phase
string message
```

## `UpdatePlanningScene` Behavior

- Resolve the saved map directory from `map_id`.
- Do not depend on scan node memory.
- If `map_id` is not found, return `success=false` with a clear message.
- If `clear_existing` is true, clear existing workspace collision objects or OctoMap data before applying the new result.
- Prefer existing logic in `octomap_to_moveit.py` and `collision_to_moveit.py`.

## Dummy Implementation Order

1. Make interfaces build first.
2. Add `ScanWorkspaceSummary.msg`.
3. Update `ScanWorkspace.action`.
4. Update `UpdatePlanningScene.action`.
5. Create dummy `ScanWorkspace` Action Server.
6. Create dummy `UpdatePlanningScene` Action Server.

Dummy `ScanWorkspace` must verify:

- Action goal can be received.
- Empty `camera_name` falls back to `hand_camera`.
- Unknown `camera_name` is rejected.
- Concurrent scan is rejected with `rejected_busy`.
- Feedback is published during `duration_sec`.
- Cancel request is handled.
- `map_id` can be generated.
- Map save directory can be created.
- Dummy `octomap.bt`, `summary.json`, and `metadata.json` can be saved.
- `summary_json` and `ScanWorkspaceSummary` can be returned.

Dummy `UpdatePlanningScene` must verify:

- Saved map directory can be resolved from `map_id`.
- Unknown `map_id` returns `success=false`.
- Feedback phases `loading_map`, `clearing`, `applying_octomap`, and `applying_collision_objects` can be published.
- Cancel request is handled.

## Connecting To Existing Implementation

After the dummy Action path is stable, connect `scan_workspace_action_server.py` to existing recognition logic.

Prefer these existing files:

```text
autonomous_recognition.py
mapping_move.py
convert_octomap.py
octomap_to_moveit.py
collision_to_moveit.py
yolo_detection.py
```

If existing files are script-style and hard to call from an Action Server, extract only the smallest callable function needed. Do not do broad refactors.

## Utils Implementation

### `scan_workspace_client.py`

- Hide `ScanWorkspace` Action Client boilerplate.
- Allow goal fields: `request_id`, `workspace_id`, `camera_name`, `duration_sec`.
- Convert result to a Python dict.

### `update_planning_scene_client.py`

- Hide `UpdatePlanningScene` Action Client boilerplate.
- Allow goal fields: `request_id`, `map_id`, `clear_existing`.
- Convert result to a Python dict.

### `rag_tool.py`

Provide the entry points used by `rag_factory_specific_task_agent`.

Target API:

```python
scan_workspace(workspace_id, duration_sec, camera_name="hand_camera")
update_planning_scene(map_id, clear_existing=True)
```

Example:

```python
from octomap_workspace_recognition2_utils.rag_tool import (
    scan_workspace,
    update_planning_scene,
)

scan_result = scan_workspace(
    workspace_id="front_workspace",
    duration_sec=5.0,
    camera_name="hand_camera",
)

if scan_result["summary"]["obstacle_detected"]:
    scene_result = update_planning_scene(
        map_id=scan_result["map_id"],
        clear_existing=True,
    )
```

## Build And Verification

From `ros2_ws`:

```bash
colcon list --base-paths src/octomap_workspace_recognition2
```

```bash
colcon build --base-paths src/octomap_workspace_recognition2 \
  --packages-select \
  octomap_workspace_recognition2_interfaces \
  octomap_workspace_recognition2_utils \
  octomap_workspace_recognition2
```

If the package layout changes, remove the affected package build and install directories before rebuilding.

## Acceptance Criteria

- `colcon build` passes.
- `ScanWorkspace` Action returns a dummy result.
- `ScanWorkspace` rejects concurrent scan execution.
- Empty `camera_name` uses `hand_camera`.
- Unknown `camera_name` is rejected.
- Scan result directory is created.
- `octomap.bt`, `summary.json`, and `metadata.json` are saved.
- `UpdatePlanningScene` resolves a saved map from `map_id`.
- `rag_tool.py` can call `scan_workspace` and `update_planning_scene`.
- `rag_factory_specific_task_agent` does not directly import runtime package internals.
