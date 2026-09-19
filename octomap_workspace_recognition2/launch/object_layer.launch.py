from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# xArm6 のグリッパ（xarm_gripper）は指が可動関節なので、手先リンクの剛体クラスタには
# 入らない。掴んだ物体は指に触れるので、許可リンクとして列挙する。
XARM_GRIPPER_TOUCH_LINKS = [
    "left_finger", "right_finger",
    "left_inner_knuckle", "right_inner_knuckle",
    "left_outer_knuckle", "right_outer_knuckle",
]


def generate_launch_description():
    attach_link = DeclareLaunchArgument("default_attach_link", default_value="link_tcp")
    planning_frame = DeclareLaunchArgument("planning_frame", default_value="world")
    server = Node(
        package="octomap_workspace_recognition2",
        executable="object_layer_action_server",
        name="object_layer_action_server",
        output="screen",
        parameters=[{
            "default_attach_link": LaunchConfiguration("default_attach_link"),
            "planning_frame": LaunchConfiguration("planning_frame"),
            "extra_touch_links": XARM_GRIPPER_TOUCH_LINKS,
        }],
    )
    return LaunchDescription([attach_link, planning_frame, server])
