from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    hw_ns = LaunchConfiguration("hw_ns")

    xarm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_moveit_config"),
                "launch",
                "xarm6_moveit_realmove.launch.py",
            ])
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "hw_ns": hw_ns,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.195",
            description="xArm6 controller IP address.",
        ),
        DeclareLaunchArgument(
            "hw_ns",
            default_value="xarm",
            description="xArm hardware namespace.",
        ),
        xarm_moveit,
    ])
