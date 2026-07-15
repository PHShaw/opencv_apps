from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='people_detect'),
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
            'hit_threshold', default_value='0',
            description='Threshold for the distance between features and SVM classifying plane'),
        DeclareLaunchArgument(
            'win_stride', default_value='8', description='Window stride. It must be a multiple of block stride.'),
        DeclareLaunchArgument(
            'padding', default_value='32',
            description='Mock parameter to keep the CPU interface compatibility. It must be (0,0).'),
        DeclareLaunchArgument(
            'scale0', default_value='1.05', description='Coefficient of the detection window increase.'),
        DeclareLaunchArgument(
            'group_threshold', default_value='2',
            description='Coefficient to regulate the similarity threshold. When detected, some objects can be '
                        'covered by many rectangles. 0 means not to perform grouping.'),

        Node(
            package='opencv_apps',
            executable='people_detect',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'hit_threshold': LaunchConfiguration('hit_threshold'),
                'win_stride': LaunchConfiguration('win_stride'),
                'padding': LaunchConfiguration('padding'),
                'scale0': LaunchConfiguration('scale0'),
                'group_threshold': LaunchConfiguration('group_threshold'),
            }],
        ),
    ])
