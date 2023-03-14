#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node


from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import RotateAngle
import time
import socket

CREATE_IP = '192.168.1.117'
CREATE_PORT = 8883

# serial communication with the ESP32 using TCP connection
class TCPserver():
    def __init__(self, IP, PORT):
        try: 
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print("Failed to create socket.")
            sys.exit()
        print("Socket created.")

        self.client.connect((IP, PORT))
        print("Socket connected to" + CREATE_IP)
    
    def read(self):
        try:
            msg = self.client.rcv(1024).decode("ascii")
        except socket.error:
            print("Failed to read data")
        return msg
    
    def write(self, string):
        try: 
            self.client.send(bytes(string.encode))
        except socket.error:
            print("Failed to send data.")

    def close(self):
        self.client.close()


# Create a publisher that communicates with the robot 
class Rotate(Node):
    def __init__(self):
        super().__init__('rotate')
        self.tcp = TCPserver(CREATE_IP, CREATE_PORT)
        # publish to cmd_vel topic with msg Twist and queue of 10
        self._action_client_rotate_angle = ActionClient(self, RotateAngle, 'rotate_angle')


    def send_goal(self, angle):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle

        self._action_client_rotate_angle.wait_for_server()

        return self._action_client_rotate_angle.send_goal_async(goal_msg)
    def angle_detection(self, data):
        r1 = data.find('(')
        r2 = data.find(')')
        newdata = data[r1+1:r2]
        print(newdata)
        self.send_goal(newdata)

    def get_data(self):
        data = self.tcp.read()

def main(args = None):
    rclpy.init(args=args)
    rotate = Rotate()

    # continuosly receive data
    while True:
        rotate.get_data()
        time.sleep(0.1)
        try:
            rclpy.spin(rotate)
        except KeyboardInterrupt:
            print('\nCaught Keyboard Interrupt.')
        finally:
            print('Done')
            rclpy.shutdown()
    
if __name__ == '__main__':
    main()