import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ellie_msgs.action import Move

class ActionClient(Node):

    def __init__(self):
        super().__init__("action_client")
        self._action_client = ActionClient(self, Move, "move")

    def send_goal(self,order):
        goal_msg = Move.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
