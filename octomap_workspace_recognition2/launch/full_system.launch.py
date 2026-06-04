from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    frame_id = LaunchConfiguration("frame_id")
    use_rviz = LaunchConfiguration("use_rviz")
    map_save_dir = LaunchConfiguration("map_save_dir")
    known_cameras = LaunchConfiguration("known_cameras")

    octomap_layers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("octomap_workspace_recognition2"),
                "launch",
                "octomap_layers.launch.py",
            ])
        ),
        launch_arguments={
            "cloud_topic": cloud_topic,
            "frame_id": frame_id,
        }.items(),
    )

    action_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("octomap_workspace_recognition2"),
                "launch",
                "octomap_wp_recog_action_servers.launch.py",
            ])
        ),
        launch_arguments={
            "map_save_dir": map_save_dir,
            "known_cameras": known_cameras,
        }.items(),
    )

    octomap_to_moveit_nodes = [
        Node(
            package="octomap_workspace_recognition2",
            executable="octomap_to_moveit",
            name=f"{name}_octomap_to_moveit",
            output="screen",
            parameters=[{
                "source_topic": source_topic,
                "update_period": update_period,
                "frame_id": frame_id,
            }],
        )
        for name, source_topic, update_period in [
            ("static", "/octomap_static/octomap_binary", 1.0),
            ("semi_static", "/octomap_semi_static/octomap_binary", 0.1),
            ("dynamic", "/octomap_dynamic/octomap_binary", 0.01),
        ]
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("octomap_workspace_recognition2"),
                "rviz",
                "xarm6_octomap.rviz",
            ]),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/camera/hand_camera/depth/color/points",
            description="Input PointCloud2 topic from RealSense.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="world",
            description="Octomap frame.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2.",
        ),
        DeclareLaunchArgument(
            "map_save_dir",
            default_value="",
            description="Base directory for saving scan maps.",
        ),
        DeclareLaunchArgument(
            "known_cameras",
            default_value="hand_camera",
            description="Comma-separated list of known camera names.",
        ),
        octomap_layers,
        action_servers,
        *octomap_to_moveit_nodes,
        rviz,
    ])
