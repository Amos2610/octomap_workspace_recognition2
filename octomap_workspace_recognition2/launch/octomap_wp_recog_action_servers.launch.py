from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_save_dir = LaunchConfiguration("map_save_dir")
    known_cameras = LaunchConfiguration("known_cameras")

    scan_workspace_server = Node(
        package="octomap_workspace_recognition2",
        executable="scan_workspace_action_server",
        name="scan_workspace_action_server",
        output="screen",
        parameters=[{
            "map_save_dir": map_save_dir,
            "known_cameras": known_cameras,
        }],
    )

    update_planning_scene_server = Node(
        package="octomap_workspace_recognition2",
        executable="update_planning_scene_action_server",
        name="update_planning_scene_action_server",
        output="screen",
        parameters=[{
            "map_save_dir": map_save_dir,
        }],
    )

    register_obstacles_server = Node(
        package="octomap_workspace_recognition2",
        executable="register_obstacles_action_server",
        name="register_obstacles_action_server",
        output="screen",
    )

    move_and_scan_server = Node(
        package="octomap_workspace_recognition2",
        executable="move_and_scan_action_server",
        name="move_and_scan_action_server",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_save_dir",
            default_value="",
            description="Base directory for saving scan maps. Defaults to <package_share>/io/maps.",
        ),
        DeclareLaunchArgument(
            "known_cameras",
            default_value="hand_camera",
            description="Comma-separated list of known camera names.",
        ),
        scan_workspace_server,
        update_planning_scene_server,
        register_obstacles_server,
        move_and_scan_server,
    ])
