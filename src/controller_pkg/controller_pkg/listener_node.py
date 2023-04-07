#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# this node listens on the /uart topic 
class ListenerNode(Node):
    def __init__(self):
        super().__init__("follow_request")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # Create a subscriber to custom topic /uart
        self.subscriber = self.create_subscription(String, "/uart", self.listener_callback_uart, qos_profile=qos_profile)

    
    # listening on topic /uart to receive following request
    def listener_callback_uart(self, msg):
        serial_data = msg.data

        return serial_data

    

def main(args=None):
    rclpy.init(args=args)
    followrequest = ListenerNode()
    rclpy.spin(followrequest)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
