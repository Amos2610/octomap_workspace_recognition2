import argparse
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from xarm_utils_py import Node, XArmUtils


def _log(node, level, message):
    logger = getattr(node, "get_logger", lambda: None)()
    if logger is None:
        print(message)
        return
    getattr(logger, level)(message)


def default_config_path():
    package_share = get_package_share_directory("octomap_workspace_recognition2")
    return os.path.join(package_share, "io", "config", "scan_poses.yaml")


class MoveItMapping:
    def __init__(self, node: Node, xarm: XArmUtils, config_path: str):
        self.node = node
        self.xarm = xarm
        self.config_path = config_path

        with open(config_path, "r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream) or {}

    def execute_poses(
        self,
        pose_list_key: str,
        pipeline: str = "ompl",
        velocity_scale=None,
        accel_scale=None,
    ) -> bool:
        try:
            poses = self.config["Joint"][pose_list_key]
        except KeyError:
            _log(self.node, "error", f"Pose list not found: Joint.{pose_list_key}")
            return False

        if velocity_scale is not None:
            self._set_move_group_parameter("max_velocity_scaling_factor", velocity_scale)
        if accel_scale is not None:
            self._set_move_group_parameter("max_acceleration_scaling_factor", accel_scale)

        self.xarm.set_planning_pipeline(pipeline)
        _log(
            self.node,
            "info",
            f"Executing {len(poses)} poses from {pose_list_key} with {pipeline}",
        )

        for index, joint_values in enumerate(poses, start=1):
            joints = [float(value) for value in joint_values]
            self.xarm.set_joint_value_target(joints)
            success, _, duration, error_code = self.xarm.plan()
            if not success:
                _log(
                    self.node,
                    "error",
                    f"Planning failed at pose {index}: error={error_code}",
                )
                return False

            _log(self.node, "info", f"Plan success at pose {index}: duration={duration}")
            if not self.xarm.execute():
                _log(self.node, "error", f"Execution failed at pose {index}")
                return False

        return True

    def _set_move_group_parameter(self, key, value):
        setter = getattr(self.xarm, "set_move_group_parameter", None)
        if setter is None:
            _log(self.node, "warn", f"xarm does not expose set_move_group_parameter: {key}")
            return
        setter(key, float(value))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    parser.add_argument("--pose-key", default="mapping_static")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--move-group", default="xarm6")
    parser.add_argument("--velocity-scale", type=float, default=None)
    parser.add_argument("--accel-scale", type=float, default=None)
    args, _ = parser.parse_known_args(argv)

    node = Node("mapping_move")
    xarm = XArmUtils(node, args.move_group)
    mapper = MoveItMapping(node, xarm, args.config or default_config_path())
    ok = mapper.execute_poses(
        args.pose_key,
        pipeline=args.pipeline,
        velocity_scale=args.velocity_scale,
        accel_scale=args.accel_scale,
    )
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
