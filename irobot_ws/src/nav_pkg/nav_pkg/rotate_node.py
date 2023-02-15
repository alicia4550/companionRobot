#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RotateNode(Node):
    def __init__(self, speed):
        super().__init__("rotate_node")
        self.speed_ = speed
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.set_speed()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.set_speed)

    def set_speed(self):
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = -self.speed_
        self.publisher_cmd_vel.publish(velocity)