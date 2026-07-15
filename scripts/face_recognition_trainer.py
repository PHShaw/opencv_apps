#! /usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Yuki Furuta.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import threading
import time

import message_filters
import rclpy
from rclpy.node import Node

from opencv_apps.msg import FaceArrayStamped
from opencv_apps.srv import FaceRecognitionTrain
from sensor_msgs.msg import Image


class FaceRecognitionTrainer(Node):

    def __init__(self):
        super().__init__('face_recognition_trainer')
        queue_size = self.declare_parameter('queue_size', 100).value

        self.img_sub = message_filters.Subscriber(self, Image, 'image')
        self.face_sub = message_filters.Subscriber(self, FaceArrayStamped, 'faces')

        self.req = FaceRecognitionTrain.Request()
        self.label = ''
        self.ok = False

        self.sync = message_filters.TimeSynchronizer([self.img_sub, self.face_sub], queue_size)
        self.sync.registerCallback(self.callback)

        self.train_client = self.create_client(FaceRecognitionTrain, 'train')

    def callback(self, img, faces):
        if len(faces.faces) <= 0:
            return
        if self.ok:
            faces.faces.sort(key=lambda f: f.face.width * f.face.height)
            self.req.images.append(img)
            self.req.rects.append(faces.faces[0].face)
            self.req.labels.append(self.label)
            self.ok = False

    def run(self):
        while not self.train_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the "train" service...')

        self.label = input('Please input your name and press Enter: ')
        while len(self.label) <= 0 or input(f'Your name is {self.label}. Correct? [y/n]: ') not in ['', 'y', 'Y']:
            self.label = input('Please input your name and press Enter: ')

        input('Please stand at the center of the camera and press Enter: ')
        while True:
            self.ok = True
            while self.ok:
                print('taking picture...')
                time.sleep(1)
            if input('One more picture? [y/n]: ') not in ['', 'y', 'Y']:
                break
        print('sending to trainer...')

        future = self.train_client.call_async(self.req)
        while not future.done():
            time.sleep(0.05)
        res = future.result()
        if res.ok:
            print('OK. Trained successfully!')
        else:
            print(f'NG. Error: {res.error}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionTrainer()
    # Spin in the background so message_filters callbacks and the service
    # response keep being processed while run() blocks on input()/sleep().
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
