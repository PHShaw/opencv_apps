from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='rgb_color_filter'),
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
        DeclareLaunchArgument('r_limit_max', default_value='250', description='The maximum allowed field value Red'),
        DeclareLaunchArgument('r_limit_min', default_value='50', description='The minimum allowed field value Red'),
        DeclareLaunchArgument('g_limit_max', default_value='200', description='The maximum allowed field value Green'),
        DeclareLaunchArgument('g_limit_min', default_value='100', description='The minimum allowed field value Green'),
        DeclareLaunchArgument('b_limit_max', default_value='150', description='The maximum allowed field value Blue'),
        DeclareLaunchArgument('b_limit_min', default_value='20', description='The minimum allowed field value Blue'),

        Node(
            package='opencv_apps',
            executable='rgb_color_filter',
            name=LaunchConfiguration('node_name'),
            output='screen',
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'r_limit_max': LaunchConfiguration('r_limit_max'),
                'r_limit_min': LaunchConfiguration('r_limit_min'),
                'g_limit_max': LaunchConfiguration('g_limit_max'),
                'g_limit_min': LaunchConfiguration('g_limit_min'),
                'b_limit_max': LaunchConfiguration('b_limit_max'),
                'b_limit_min': LaunchConfiguration('b_limit_min'),
            }],
        ),
    ])
