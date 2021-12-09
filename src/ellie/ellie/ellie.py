# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from typing import OrderedDict
import rclpy
from rclpy.node import Node
import json
from pathlib import Path
import os
from ellie_msgs.msg import DetectedInfo
from ellie_msgs.srv import String
import time
import random

REALESE_TIME = 600


class Ellie(Node):

    def __init__(self):
        super().__init__("action_client")

        self.trigger = False
        with open(os.path.join(Path(__file__).parent, "behaviors.json")) as file:
            self.registed = json.load(file, object_pairs_hook=OrderedDict)

        self.get_logger().info(
            str(self.registed["ladygaga"]["eyes_animation"]))
        self._recognized_subscriber = self.create_subscription(
            DetectedInfo, "recognized_person", self.recognized_subscriber_callback, 10)
        print(self._recognized_subscriber.topic_name)
        self._speak = self.create_client(String, "speak")
        self._eyes_animation = self.create_client(String, "change_eyes_motion")
        self._execute_behavior_srv = self.create_client(
            String, "execute_behavior")
        self.client_futures = []
        self.black_list = {}

    def recognized_subscriber_callback(self, msg):
        self.get_logger().info(str(msg))
        registed_names = list(self.registed.keys())
        if len(msg.names) > 0:
            # TODO: rotate robot head
            self.get_logger().info(str(msg.names))
            black_keys = list(self.black_list.keys())
            i = random.choice([x for x in msg.names if x not in black_keys or (
                time.time()-self.black_list[x.lower]()) > REALESE_TIME])
            id = i.lower()
            self.black_list[id] = time.time()
            if id in registed_names:
                self.client_futures.append(self.send_behavior_request(
                    random.choice(self.registed[id]["behavior"])))
                self.client_futures.append(self.send_speak_request(
                    random.choice(self.registed[id]["speak"])))
                self.client_futures.append(self.send_eyes_animation_request(
                    random.choice(self.registed[id]["eyes_animation"])))

    def send_behavior_request(self, data):
        msg = String.Request()
        msg.request = data
        return self._execute_behavior_srv.call_async((msg))

    def send_speak_request(self, data):
        msg = String.Request()
        msg.request = data
        return self._speak.call_async(msg)

    def send_eyes_animation_request(self, data):
        msg = String.Request()
        msg.request = data
        return self._eyes_animation.call_async(msg)


def main(args=None):
    rclpy.init(args=args)
    ellie = Ellie()
    time.sleep(5)
    while rclpy.ok():
        rclpy.spin_once(ellie)

        for i in ellie.client_futures:
            if i.done():
                ellie.get_logger().info(f"Response : {i.result()}")
            else:
                rclpy.spin_until_future_complete(ellie, i, timeout_sec=60)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
