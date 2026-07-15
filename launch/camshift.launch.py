from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='camshift'),
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
            'histogram', default_value='[0.0, 255.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Initial histogram of tracked color object (session-only; not written back when a new '
                         'histogram is learned interactively, unlike the ROS 1 version).'),
        DeclareLaunchArgument('vmin', default_value='10', description='Min threshold of lightness.'),
        DeclareLaunchArgument('vmax', default_value='230', description='Max threshold of lightness.'),
        DeclareLaunchArgument('smin', default_value='60', description='Min value of saturation.'),

        Node(
            package='opencv_apps',
            executable='camshift',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'histogram': LaunchConfiguration('histogram'),
                'vmin': LaunchConfiguration('vmin'),
                'vmax': LaunchConfiguration('vmax'),
                'smin': LaunchConfiguration('smin'),
            }],
        ),
    ])
