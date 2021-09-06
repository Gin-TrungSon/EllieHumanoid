# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ellie_eyes",
            namespace="ellie",
            executable="start",
            name="eyes"
        ),
        Node(
            package="ellie_arm",
            namespace="ellie",
            executable="start",
            name="arm"
        ),
        Node(
            package="ellie_screens",
            namespace="ellie",
            executable="start_head",
            name="head_screen"
        ),
        Node(
            package="ellie_screens",
            namespace="ellie",
            executable="start_brust",
            name="brust_screen"
        ),
        Node(
            package="ellie",
            namespace="ellie",
            executable="start",
            name="ellie"
        ),
        Node(
            package="ellie_voice",
            namespace="ellie",
            executable="start",
            name="voice"
        )
    ])
