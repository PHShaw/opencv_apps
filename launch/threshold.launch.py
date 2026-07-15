from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='threshold'),
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
            'apply_otsu', default_value='false',
            description="Decide applying Otsu's algorithm threshold. If this value is true, threshold value "
                        'will be ignored'),
        DeclareLaunchArgument(
            'threshold_type', default_value='0',
            description='Specify threshold types. 0: Binary threshold, 1: Binary inverted threshold, '
                        '2: Threshold truncated, 3: Threshold to zero, 4: Threshold to zero inverted'),
        DeclareLaunchArgument('threshold', default_value='150', description='threshold value'),
        DeclareLaunchArgument(
            'max_binary', default_value='255',
            description='maximum value to use with the 0: Binary thresholding and 1: Binary inverted '
                        'thresholding types.'),

        Node(
            package='opencv_apps',
            executable='threshold',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'threshold_type': LaunchConfiguration('threshold_type'),
                'apply_otsu': LaunchConfiguration('apply_otsu'),
                'threshold': LaunchConfiguration('threshold'),
                'max_binary': LaunchConfiguration('max_binary'),
            }],
        ),
    ])
