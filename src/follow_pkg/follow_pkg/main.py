#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')
        self.ir_intensity_front_center_left = 0
        self.ir_intensity_front_center_right = 0
        self.ir_intensity_front_left = 0
        self.ir_intensity_front_right = 0
        self.ir_intensity_left = 0
        self.ir_intensity_right = 0
        self.ir_intensity_side_left = 0

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_callback)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            qos_profile=qos_profile
        )

    def ir_callback(self, msg):
        for data in msg.readings:
            # self.get_logger().info(str(data.header.frame_id))
            if data.header.frame_id == 'ir_intensity_front_center_right':
                self.ir_intensity_front_center_right = data.value
            if data.header.frame_id == 'ir_intensity_front_center_left':
                self.ir_intensity_front_center_left = data.value
            if data.header.frame_id == 'ir_intensity_front_right':
                self.ir_intensity_front_right = data.value
            if data.header.frame_id == 'ir_intensity_front_left':
                self.ir_intensity_front_left = data.value
            if data.header.frame_id == 'ir_intensity_right':
                self.ir_intensity_right = data.value
            if data.header.frame_id == 'ir_intensity_left':
                self.ir_intensity_left = data.value
            if data.header.frame_id == 'ir_intensity_side_left':
                self.ir_intensity_side_left = data.value

    def cmd_callback(self):
        ir_front = (self.ir_intensity_front_center_left + self.ir_intensity_front_center_right) * 0.5
        ir_left = (self.ir_intensity_front_left + self.ir_intensity_left + self.ir_intensity_side_left) / 3.0
        ir_right = (self.ir_intensity_front_right + self.ir_intensity_right) * 0.5
        cmd_twist = Twist()

        if ir_front > 10 and ir_front > ir_right and ir_front > ir_left:
            err = 100 - ir_front
            cmd_twist.linear.x = err * 0.001
            cmd_twist.linear.x = min(0.5, cmd_twist.linear.x)
            cmd_twist.linear.x = max(-0.5, cmd_twist.linear.x)
        elif ir_right > 10:
            err = 100 - ir_right
            cmd_twist.angular.z = -1.5
        elif ir_left > 10:
            err = 100 - ir_left
            cmd_twist.angular.z = 1.5
        
        self.publisher_.publish(cmd_twist)


def main(args=None):
    rclpy.init(args=args)
    follow_node = FollowNode()
    rclpy.spin(follow_node)
    follow_node.destroy_node()
    rclpy.shutdown()