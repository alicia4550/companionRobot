#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class GetOdomNode(Node):
    def __init__(self):
        super().__init__('get_odom_node')
        self.subscription = self.create_subscription(Odometry, '/odom', self.position_callback, qos_profile_sensor_data)
        self.position_ = Point()
        self.orientation_ = Quaternion()

    def position_callback(self, msg):
        self.position_ = msg.pose.pose.position
        self.orientation_ = msg.pose.pose.orientation
        self.get_logger().info("Current angle: x={0:.2f}, y={1:.2f}, z={2:.2f}".format(self.orientation_.x, self.orientation_.y, self.orientation_.z))
        self.get_logger().info("Current position: x={0:.2f}, y={1:.2f}".format(self.position_.x, self.position_.y))

    def euler_from_quaternion(self):
        x = self.orientation_.x
        y = self.orientation_.y
        z = self.orientation_.z
        w = self.orientation_.w

        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z