#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs import wheelVels
from geometry_msgs import Twist
from geometry_msgs import Vector3

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import Parameter

import socket 
import time

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

        self.client.connect(IP, PORT)
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
class LineFollow():
    def __init__(self):
        super().__init__('linefollow')
        self.tcp = TCPserver(CREATE_IP, CREATE_PORT)
        # publish to cmd_vel topic with msg Twist and queue of 10
        self.wheels_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.wheels = Twist() # message type
        self.linear = Vector3()
        self.angular = Vector3()

        # override safety params
        self.client = self.create_client(SetParameters, '/motion_control/set_parameters')

    def set_params(self):
        request = SetParameters.Request()
        param = Parameter()
        param.name = 'safety_override'
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'full'
        
        request.parameters.append(param)

        self.client.wait_for_service()
        self.future = self.client.call_async(request)

    def send_speed(self, linear_speed, angular_speed):
        self.linear.x = float(linear_speed)
        self.linear.y = float(linear_speed)
        self.linear.z = float(linear_speed)

        self.angular.x = float(angular_speed)
        self.angular.y = float(angular_speed)
        self.angular.z = float(angular_speed)

        self.wheels.linear = self.linear 
        self.wheels.angular = self.angular

        self.wheels_publisher.publish(self.wheels)

    def get_data(self):
        data = self.tcp.read()

def main(args = None):
    rclpy.init(args=args)
    linefollow = LineFollow()

    linefollow.set_params() #override safety

    # continuosly receive data
    while True:
        linefollow.get_data()
        time.sleep(0.1)
        try:
            rclpy.spin(linefollow)
        except KeyboardInterrupt:
            print('\nCaught Keyboard Interrupt.')
        finally:
            print('Done')
            rclpy.shutdown()
    
if __name__ == '__main__':
    main()


    


