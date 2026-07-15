from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='hough_lines'),
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
            'hough_type', default_value='0',
            description='Specify Hough transform type, 0 for Standard Hough Detection and 1 for Probabilistic '
                        'Hough Detection'),
        DeclareLaunchArgument(
            'threshold', default_value='150', description='Threshold value for hough line detection'),
        DeclareLaunchArgument(
            'rho', default_value='1.0', description='The resolution of the parameter r in pixels.'),
        DeclareLaunchArgument(
            'theta', default_value='1.0', description='The resolution of the parameter theta in radians.'),
        DeclareLaunchArgument(
            'minLineLength', default_value='30', description='The minimum number of points that can form a line.'),
        # NOTE: the ROS 1 launch file had this argument (and the param it fed) misspelled as
        # "maxLineGrap", which silently never reached the real "maxLineGap" node parameter.
        # Fixed here so the launch argument actually works.
        DeclareLaunchArgument(
            'maxLineGap', default_value='10',
            description='The maximum gap between two points to be considered in the same line.'),

        Node(
            package='opencv_apps',
            executable='hough_lines',
            name=LaunchConfiguration('node_name'),
            remappings=[('image', LaunchConfiguration('image'))],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'hough_type': LaunchConfiguration('hough_type'),
                'threshold': LaunchConfiguration('threshold'),
                'rho': LaunchConfiguration('rho'),
                'theta': LaunchConfiguration('theta'),
                'minLineLength': LaunchConfiguration('minLineLength'),
                'maxLineGap': LaunchConfiguration('maxLineGap'),
            }],
        ),
    ])
