#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
from rclpy.qos import qos_profile_sensor_data

class GetIrNode(Node):
    def __init__(self):
        super().__init__('get_ir_node')
        self.subscription = self.create_subscription(IrIntensityVector, 'ir_intensity', self.position_callback, qos_profile_sensor_data)
        self.intensity_ = [0] * 7

    def position_callback(self, msg):
        readings = msg.readings
        for i in range(len(readings)):
            self.intensity_[i] = readings[i].value
        print(self.intensity_)