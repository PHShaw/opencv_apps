from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='lk_flow'),
        DeclareLaunchArgument(
            'image', default_value='image',
            description='The image topic. Should be remapped to the name of the real image topic.'),
        DeclareLaunchArgument(
            'use_camera_info', default_value='false',
            description='Indicates that the camera_info topic should be subscribed to to get the default '
                         'input_frame_id. Otherwise the frame from the image message will be used.'),
        DeclareLaunchArgument(
            'debug_view', default_value='true',
            description='Specify whether the node displays a window to show edge image'),
        DeclareLaunchArgument(
            'queue_size', default_value='3', description='Specify queue_size of input image subscribers'),
        DeclareLaunchArgument(
            'quality_level', default_value='0.01',
            description='Parameter characterizing the minimal acceptance quality of image corners.'),
        DeclareLaunchArgument(
            'min_distance', default_value='10',
            description='Minimum possible Euclidean distance between the returned corners.'),
        DeclareLaunchArgument(
            'block_size', default_value='3',
            description='Size of an average block for computing a derivative covariation matrix over each '
                        'pixel neighborhood.'),
        DeclareLaunchArgument('harris_k', default_value='0.04', description='Free parameter of the Harris detector'),

        Node(
            package='opencv_apps',
            executable='lk_flow',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'quality_level': LaunchConfiguration('quality_level'),
                'min_distance': LaunchConfiguration('min_distance'),
                # NOTE: the ROS 1 launch file misspelled this param as "blcok_size" when wiring
                # it to the node, so the block_size launch argument silently never took effect.
                'block_size': LaunchConfiguration('block_size'),
                'harris_k': LaunchConfiguration('harris_k'),
            }],
        ),
    ])
