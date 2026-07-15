import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_face_detection', default_value='true'),
        DeclareLaunchArgument('launch_trainer', default_value='true'),
        DeclareLaunchArgument('debug_view', default_value='true'),
        DeclareLaunchArgument(
            'queue_size', default_value='100', description='Specify queue_size of input image subscribers'),
        DeclareLaunchArgument('image', default_value='image'),
        DeclareLaunchArgument('data_dir', default_value='~/.ros/opencv_apps/face_data'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('opencv_apps'), 'launch', 'face_detection.launch.py')),
            condition=IfCondition(LaunchConfiguration('launch_face_detection')),
            launch_arguments={
                'image': LaunchConfiguration('image'),
                'debug_view': LaunchConfiguration('debug_view'),
                'node_name': 'face_detection',
            }.items(),
        ),

        Node(
            package='opencv_apps',
            executable='face_recognition',
            name='face_recognition',
            output='screen',
            parameters=[{
                'data_dir': LaunchConfiguration('data_dir'),
                'queue_size': LaunchConfiguration('queue_size'),
            }],
            remappings=[
                ('image', LaunchConfiguration('image')),
                ('faces', 'face_detection/faces'),
            ],
        ),

        Node(
            package='opencv_apps',
            executable='face_recognition_trainer.py',
            name='face_recognition_trainer',
            condition=IfCondition(LaunchConfiguration('launch_trainer')),
            prefix='xterm -fn 12x24 -e',
            respawn=True,
            remappings=[
                ('image', LaunchConfiguration('image')),
                ('faces', 'face_detection/faces'),
                ('train', 'face_recognition/train'),
            ],
        ),

        Node(
            package='image_view',
            executable='image_view',
            name='debug_image_viewer',
            condition=IfCondition(LaunchConfiguration('debug_view')),
            remappings=[('image', 'face_recognition/debug_image')],
        ),
    ])
