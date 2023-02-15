#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import DriveDistance

class DriveDistanceNode(Node):
    def __init__(self):
        super().__init__("drive_distance_node")
        self._action_client_drive_distance = ActionClient(self, DriveDistance, 'drive_distance')

    def send_goal(self):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.5
        goal_msg.max_translation_speed = 0.15

        self._action_client_drive_distance.wait_for_server()

        return self._action_client_drive_distance.send_goal_async(goal_msg)