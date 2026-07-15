from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='smoothing'),
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
            'filter_type', default_value='0',
            description='Smoothing Filter Methods. 0:Homogeneous blur, 1:Gaussian blur, 2:Median blur, '
                         '3:Bilateral Filter.'),
        DeclareLaunchArgument(
            'kernel_size', default_value='7',
            description='Size of the kernel (only one because we use a square window). Must be odd.'),

        Node(
            package='opencv_apps',
            executable='smoothing',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'filter_type': LaunchConfiguration('filter_type'),
                'kernel_size': LaunchConfiguration('kernel_size'),
            }],
        ),
    ])
