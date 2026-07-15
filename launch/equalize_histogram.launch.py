from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='equalize_histogram'),
        DeclareLaunchArgument(
            'image', default_value='image',
            description='The image topic. Should be remapped to the name of the real image topic.'),
        DeclareLaunchArgument(
            'use_camera_info', default_value='false',
            description='Indicates that the camera_info topic should be subscribed to to get the default '
                         'input_frame_id. Otherwise the frame from the image message will be used.'),
        DeclareLaunchArgument(
            'debug_view', default_value='true', description='Specify whether the node displays a window to show image'),
        DeclareLaunchArgument(
            'queue_size', default_value='3', description='Specify queue_size of input image subscribers'),
        DeclareLaunchArgument(
            'histogram_equalization_type', default_value='0',
            description='Specify histogram equalization method. 0: Default, 1: Clahe'),
        DeclareLaunchArgument(
            'clahe_clip_limit', default_value='255.0', description='The maximum allowed field value Clip Limit'),
        DeclareLaunchArgument(
            'clahe_tile_size_x', default_value='300', description='The maximum allowed field value Threshold X Limit'),
        DeclareLaunchArgument(
            'clahe_tile_size_y', default_value='300', description='The maximum allowed field value Threshold Y Limit'),
        DeclareLaunchArgument('use_opencl', default_value='true', description='Enable or disable OpenCL'),

        Node(
            package='opencv_apps',
            executable='equalize_histogram',
            name=LaunchConfiguration('node_name'),
            output='screen',
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'clahe_clip_limit': LaunchConfiguration('clahe_clip_limit'),
                'clahe_tile_size_x': LaunchConfiguration('clahe_tile_size_x'),
                'clahe_tile_size_y': LaunchConfiguration('clahe_tile_size_y'),
                'use_opencl': LaunchConfiguration('use_opencl'),
                'histogram_equalization_type': LaunchConfiguration('histogram_equalization_type'),
            }],
        ),
    ])
