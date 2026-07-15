from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='lab_color_filter'),
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
        DeclareLaunchArgument('l_limit_max', default_value='255', description='The maximum allowed field value L*'),
        DeclareLaunchArgument('l_limit_min', default_value='50', description='The minimum allowed field value L*'),
        DeclareLaunchArgument('a_limit_max', default_value='250', description='The maximum allowed field value a*'),
        DeclareLaunchArgument('a_limit_min', default_value='0', description='The minimum allowed field value a*'),
        DeclareLaunchArgument('b_limit_max', default_value='255', description='The maximum allowed field value b*'),
        DeclareLaunchArgument('b_limit_min', default_value='150', description='The minimum allowed field value b*'),

        Node(
            package='opencv_apps',
            executable='lab_color_filter',
            name=LaunchConfiguration('node_name'),
            output='screen',
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'l_limit_max': LaunchConfiguration('l_limit_max'),
                'l_limit_min': LaunchConfiguration('l_limit_min'),
                'a_limit_max': LaunchConfiguration('a_limit_max'),
                'a_limit_min': LaunchConfiguration('a_limit_min'),
                'b_limit_max': LaunchConfiguration('b_limit_max'),
                'b_limit_min': LaunchConfiguration('b_limit_min'),
            }],
        ),
    ])
