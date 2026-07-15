from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='adding_images'),
        DeclareLaunchArgument(
            'image1', default_value='image1',
            description='The image topic. Should be remapped to the name of the real image topic.'),
        DeclareLaunchArgument(
            'image2', default_value='image2',
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
        DeclareLaunchArgument('alpha', default_value='0.5', description='Weight of the first array elements'),
        DeclareLaunchArgument('gamma', default_value='0', description='Scalar added to each sum'),
        DeclareLaunchArgument(
            'approximate_sync', default_value='false',
            description='Flag of approximate_sync. If this value is false, image1 and image2 synchronize '
                         'exact time.'),

        Node(
            package='opencv_apps',
            executable='adding_images',
            name=LaunchConfiguration('node_name'),
            remappings=[
                ('image1', LaunchConfiguration('image1')),
                ('image2', LaunchConfiguration('image2')),
            ],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'alpha': LaunchConfiguration('alpha'),
                'gamma': LaunchConfiguration('gamma'),
                'approximate_sync': LaunchConfiguration('approximate_sync'),
            }],
        ),
    ])
