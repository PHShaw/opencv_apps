"""launch_testing smoke test for the find_contours node.

See test_launch_edge_detection.py for the pattern this follows.
"""
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


@pytest.mark.launch_test
def generate_test_description():
    find_contours_node = launch_ros.actions.Node(
        package='opencv_apps',
        executable='find_contours',
        name='find_contours',
        parameters=[{'debug_view': False}],
        remappings=[('image', 'test_image')],
    )
    return launch.LaunchDescription([
        find_contours_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'find_contours_node': find_contours_node}


class TestFindContoursOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_find_contours_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_output_image_published(self, proc_output):
        received = []
        self.node.create_subscription(Image, '/find_contours/image', lambda msg: received.append(msg), 10)
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

        self.assertGreater(len(received), 0, 'find_contours never published a processed image')
