"""Example of ROS 2 node composition: rgb_color_filter -> contour_moments.

Both nodes are loaded as rclcpp_components into a single component container
process (rather than run as two separate `ros2 run` processes, as every
other launch file in this package does). Composition lets rclcpp pass the
image between them via intra-process communication when
`use_intra_process_comms` is enabled, skipping the usual
serialize/publish/subscribe/deserialize round trip for local messages.

rgb_color_filter thresholds the input image to a binary mask on its
"~/image" output; contour_moments takes that mask as its own "image" input
and publishes the detected contours' moments on "~/moments".

Usage:
  ros2 launch opencv_apps example_composition_rgb_contour_moments.launch.py \\
      image:=/camera/image_raw

Inspect the pipeline once running:
  ros2 component list
  ros2 topic echo /contour_moments/moments
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image', default_value='image',
            description='The camera image topic. Should be remapped to the name of the real image topic.'),
        DeclareLaunchArgument(
            'debug_view', default_value='true',
            description='Specify whether the node displays a window to show edge image'),
        DeclareLaunchArgument(
            'container_name', default_value='opencv_apps_container',
            description='Name of the component container these nodes are loaded into.'),

        # debug_view is forced off for both nodes: cv::imshow is not safe to
        # run from more than one thread, so a composed debug-view node needs
        # a single-threaded container (component_container, used below) --
        # never component_container_mt if any loaded node has debug_view:=true.
        DeclareLaunchArgument(
            'r_limit_min', default_value='0', description='The minimum allowed field value Red'),
        DeclareLaunchArgument(
            'r_limit_max', default_value='255', description='The maximum allowed field value Red'),
        DeclareLaunchArgument(
            'g_limit_min', default_value='0', description='The minimum allowed field value Green'),
        DeclareLaunchArgument(
            'g_limit_max', default_value='100', description='The maximum allowed field value Green'),
        DeclareLaunchArgument(
            'b_limit_min', default_value='0', description='The minimum allowed field value Blue'),
        DeclareLaunchArgument(
            'b_limit_max', default_value='100', description='The maximum allowed field value Blue'),
        DeclareLaunchArgument(
            'filter_type', default_value='0',
            description='Smoothing Filter Methods. 0:Homogeneous blur, 1:Gaussian blur, 2:Median blur, '
                         '3:Bilateral Filter.'),
        DeclareLaunchArgument(
            'kernel_size', default_value='7',
            description='Size of the kernel (only one because we use a square window). Must be odd.'),
        DeclareLaunchArgument(
            'canny_low_threshold', default_value='100',
            description='Canny edge low threshold used by contour_moments to find contours in the mask.'),

        ComposableNodeContainer(
            name=LaunchConfiguration('container_name'),
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='opencv_apps',
                    plugin='opencv_apps::RGBColorFilterNode',
                    name='rgb_color_filter',
                    parameters=[{
                        'debug_view': LaunchConfiguration('debug_view'),
                        'r_limit_min': LaunchConfiguration('r_limit_min'),
                        'r_limit_max': LaunchConfiguration('r_limit_max'),
                        'g_limit_min': LaunchConfiguration('g_limit_min'),
                        'g_limit_max': LaunchConfiguration('g_limit_max'),
                        'b_limit_min': LaunchConfiguration('b_limit_min'),
                        'b_limit_max': LaunchConfiguration('b_limit_max'),
                    }],
                    remappings=[('image', LaunchConfiguration('image'))],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='opencv_apps',
                    plugin='opencv_apps::SmoothingNode',
                    name='smoothing',
                    parameters=[{
                        'debug_view': LaunchConfiguration('debug_view'),
                        'filter_type': LaunchConfiguration('filter_type'),
                        'kernel_size': LaunchConfiguration('kernel_size'),
                    }],
                    # Feed contour_moments' "image" input from rgb_color_filter's
                    # "~/image" output -- this is the pipeline connection.
                    remappings=[('image', 'rgb_color_filter/image')],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='opencv_apps',
                    plugin='opencv_apps::ContourMomentsNode',
                    name='contour_moments',
                    parameters=[{
                        'debug_view': LaunchConfiguration('debug_view'),
                        'canny_low_threshold': LaunchConfiguration('canny_low_threshold'),
                    }],
                    # Feed contour_moments' "image" input from rgb_color_filter's
                    # "~/image" output -- this is the pipeline connection.
                    remappings=[('image', 'smoothing/image')],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),
    ])
