#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.ir_intensity_front_center_left = 0
        self.ir_intensity_front_center_right = 0

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.set_speed)

        self.subscription = self.create_subscription(IrIntensityVector, '/ir_intensity', self.listener_callback_ir, qos_profile_sensor_data)

    def listener_callback_ir(self, msg):
        for data in msg.readings:
            if data.header.frame_id == 'ir_intensity_front_center_right':
                self.ir_intensity_front_center_right = data.value
            if data.header.frame_id == 'ir_intensity_front_center_left':
                self.ir_intensity_front_center_left = data.value

    def set_speed(self):
        ir_val = (self.ir_intensity_front_center_left + self.ir_intensity_front_center_right) * 0.5
        cmd_twist = Twist()
        if ir_val > 10:
            err = 100 - ir_val
            print('err:', err)
            cmd_twist.linear.x = err * 0.001
            cmd_twist.linear.x = min(0.5, cmd_twist.linear.x)
            cmd_twist.linear.x = max(-0.5, cmd_twist.linear.x)
            self.publisher_.publish(cmd_twist)
            print('cmd_twist published:', cmd_twist)

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()