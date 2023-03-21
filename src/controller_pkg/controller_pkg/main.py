#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # Create a subscriber to custom topic /uart
        self.subscriber = self.create_subscription(String, "/uart", self.listener_callback_uart, qos_profile=qos_profile)

        # start a timer
        self.start_timer = time.time()
    

    def listener_callback_uart(self, msg):
        serial_data = msg.data

        
        if (serial_data == '3'):
            print("time to follow")
            self.start_timer = time.time() # reset timer
        
        # if no follow request and timer is up to 5 minutes
        elif ((time.time() - self.start_timer) > 300):
            print("time to roam")
            self.start_timer = time.time() # reset timer
    

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

