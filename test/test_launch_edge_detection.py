"""launch_testing smoke test for the edge_detection node.

Starts edge_detection plus a small in-test image publisher, then asserts
that edge_detection republishes a processed image within a timeout. This is
a template for extending coverage to the rest of the package's nodes --
see the port's plan notes for why the full ROS 1 rostest suite (test/*.test)
was not ported 1:1.
"""
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


@pytest.mark.launch_test
def generate_test_description():
    edge_detection_node = launch_ros.actions.Node(
        package='opencv_apps',
        executable='edge_detection',
        name='edge_detection',
        parameters=[{'debug_view': False}],
        remappings=[('image', 'test_image')],
    )
    return launch.LaunchDescription([
        edge_detection_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'edge_detection_node': edge_detection_node}


class TestEdgeDetectionOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_edge_detection_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_output_image_published(self, proc_output):
        received = []
        self.node.create_subscription(Image, '/edge_detection/image', lambda msg: received.append(msg), 10)
        pub = self.node.create_publisher(Image, '/test_image', 10)

        image = Image()
        image.height = 64
        image.width = 64
        image.encoding = 'bgr8'
        image.step = image.width * 3
        image.data = bytes([0] * (image.step * image.height))

        end_time = self.node.get_clock().now().nanoseconds + 10 * 10**9
        while self.node.get_clock().now().nanoseconds < end_time and not received:
            pub.publish(image)
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertGreater(len(received), 0, 'edge_detection never published a processed image')
