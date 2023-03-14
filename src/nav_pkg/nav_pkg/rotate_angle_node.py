#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import RotateAngle

class RotateAngleNode(Node):
    def __init__(self):
        super().__init__("rotate_angle_node")
        self._action_client_rotate_angle = ActionClient(self, RotateAngle, 'rotate_angle')

    def send_goal(self, angle):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle

        self._action_client_rotate_angle.wait_for_server()

        return self._action_client_rotate_angle.send_goal_async(goal_msg)