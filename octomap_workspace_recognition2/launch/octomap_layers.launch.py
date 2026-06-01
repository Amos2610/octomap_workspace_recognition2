from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def octomap_server_node(
    *,
    namespace,
    name,
    cloud_topic,
    frame_id,
    resolution,
    max_range,
    min_range=None,
):
    parameters = {
        'resolution': ParameterValue(resolution, value_type=float),
        'frame_id': ParameterValue(frame_id, value_type=str),
        'sensor_model.max_range': ParameterValue(max_range, value_type=float),
    }

    if min_range is not None:
        parameters['sensor_model.min_range'] = ParameterValue(
            min_range,
            value_type=float,
        )

    return Node(
        package='octomap_server',
        executable='octomap_server_node',
        namespace=namespace,
        name=name,
        output='screen',
        parameters=[parameters],
        remappings=[
            ('cloud_in', cloud_topic),
        ],
    )


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    frame_id = LaunchConfiguration('frame_id')

    static_res = LaunchConfiguration('static_res')
    static_max_range = LaunchConfiguration('static_max_range')
    static_min_range = LaunchConfiguration('static_min_range')

    semi_static_res = LaunchConfiguration('semi_static_res')
    semi_static_max_range = LaunchConfiguration('semi_static_max_range')
    semi_static_min_range = LaunchConfiguration('semi_static_min_range')

    dynamic_res = LaunchConfiguration('dynamic_res')
    dynamic_max_range = LaunchConfiguration('dynamic_max_range')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/depth/color/points',
            description='Input PointCloud2 topic',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='world',
            description='Frame ID for OctoMap',
        ),

        DeclareLaunchArgument('static_res', default_value='0.10'),
        DeclareLaunchArgument('static_max_range', default_value='3.0'),
        DeclareLaunchArgument('static_min_range', default_value='1.5'),

        DeclareLaunchArgument('semi_static_res', default_value='0.05'),
        DeclareLaunchArgument('semi_static_max_range', default_value='1.5'),
        DeclareLaunchArgument('semi_static_min_range', default_value='0.7'),

        DeclareLaunchArgument('dynamic_res', default_value='0.02'),
        DeclareLaunchArgument('dynamic_max_range', default_value='0.7'),

        octomap_server_node(
            namespace='octomap_static',
            name='server_static',
            cloud_topic=cloud_topic,
            frame_id=frame_id,
            resolution=static_res,
            max_range=static_max_range,
            min_range=static_min_range,
        ),

        octomap_server_node(
            namespace='octomap_semi_static',
            name='server_semi_static',
            cloud_topic=cloud_topic,
            frame_id=frame_id,
            resolution=semi_static_res,
            max_range=semi_static_max_range,
            min_range=semi_static_min_range,
        ),

        octomap_server_node(
            namespace='octomap_dynamic',
            name='server_dynamic',
            cloud_topic=cloud_topic,
            frame_id=frame_id,
            resolution=dynamic_res,
            max_range=dynamic_max_range,
        ),
    ])
