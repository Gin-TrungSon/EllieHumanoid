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
