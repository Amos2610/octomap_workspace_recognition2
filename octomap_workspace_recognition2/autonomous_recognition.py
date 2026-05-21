import argparse
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from octomap_msgs.msg import Octomap
from rclpy.node import Node as RclpyNode
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from xarm_utils_py import Node, XArmUtils

from octomap_workspace_recognition2.mapping_move import MoveItMapping


def default_config_path():
    package_share = get_package_share_directory("octomap_workspace_recognition2")
    return os.path.join(package_share, "io", "config", "scan_poses.yaml")


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class OctomapWaiter(RclpyNode):
    def __init__(self, topic):
        super().__init__("octomap_waiter")
        self.received = False
        self.subscription = self.create_subscription(
            Octomap,
            topic,
            self._callback,
            sensor_qos(),
        )

    def _callback(self, _msg):
        self.received = True

    def wait(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        self.received = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.received:
                return True
        return False


class AutonomousWorkspaceRecognition:
    def __init__(self, config_path, move_group, octomap_topic, timeout_sec):
        self.node = Node("autonomous_recognition")
        if not rclpy.ok():
            rclpy.init()
        self.xarm = XArmUtils(self.node, move_group)
        self.mapper = MoveItMapping(self.node, self.xarm, config_path)
        self.waiter = OctomapWaiter(octomap_topic)
        self.timeout_sec = timeout_sec

    def run(self):
        phases = [
            ("mapping_static", "ompl"),
            ("mapping_semi_static", "stomp"),
            ("mapping_object", "ompl"),
        ]
        for pose_key, pipeline in phases:
            ok = self.mapper.execute_poses(pose_key, pipeline=pipeline)
            if not ok:
                return False
            if not self.waiter.wait(self.timeout_sec):
                self.waiter.get_logger().warn(
                    f"Timeout waiting for octomap after {pose_key}"
                )
        return True

    def destroy(self):
        self.waiter.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    parser.add_argument("--move-group", default="xarm6")
    parser.add_argument("--octomap-topic", default="/octomap_dynamic/octomap_binary")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args, _ = parser.parse_known_args(argv)

    app = AutonomousWorkspaceRecognition(
        args.config or default_config_path(),
        args.move_group,
        args.octomap_topic,
        args.timeout_sec,
    )
    try:
        ok = app.run()
    finally:
        app.destroy()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
