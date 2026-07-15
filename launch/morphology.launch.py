from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='morphology'),
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
            'morph_operator', default_value='0',
            description='Morphology Operation Methods 0: Opening, 1: Closing, 2: Gradient, 3: Top Hat, '
                         '4: Black Hat'),
        DeclareLaunchArgument(
            'morph_element', default_value='0', description='Type of the kernel 0: Rect, 1: Cross, 2: Ellipse'),
        DeclareLaunchArgument('morph_size', default_value='0', description='Size of the kernel'),

        Node(
            package='opencv_apps',
            executable='morphology',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'morph_operator': LaunchConfiguration('morph_operator'),
                'morph_element': LaunchConfiguration('morph_element'),
                'morph_size': LaunchConfiguration('morph_size'),
            }],
        ),
    ])
