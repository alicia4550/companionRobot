# #!/usr/bin/env python3

# import sys
# import rclpy
# from rclpy.node import Node


# from rclpy.action import ActionClient
# from rclpy.node import Node
# from irobot_create_msgs.action import RotateAngle
# import time
# import socket

# CREATE_IP = '192.168.1.162'
# CREATE_PORT = 8883

# # serial communication with the ESP32 using TCP connection
# class TCPserver():
#     def __init__(self, IP, PORT):
#         try: 
#             self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         except socket.error:
#             print("Failed to create socket.")
#             sys.exit()
#         print("Socket created.")

#         self.client.connect((IP, PORT))
#         print("Socket connected to " + CREATE_IP)
    
#     def read(self):
#         try:
#             msg = self.client.recv(1024).decode('ascii')
#         except socket.error:
#             print("Failed to read data")
#         return msg
    
#     def write(self, string):
#         try: 
#             self.client.send(bytes(string.encode))
#         except socket.error:
#             print("Failed to send data.")

#     def close(self):
#         self.client.close()


# # Create a publisher that communicates with the robot 
# # class Rotate(Node):
# #     def __init__(self):
# #         super().__init__('rotate')
# #         self.tcp = TCPserver(CREATE_IP, CREATE_PORT)
# #         # publish to cmd_vel topic with msg Twist and queue of 10
# #         self._action_client_rotate_angle = ActionClient(self, RotateAngle, 'rotate_angle')


# #     def send_goal(self, angle):
# #         goal_msg = RotateAngle.Goal()
# #         goal_msg.angle = angle

# #         self._action_client_rotate_angle.wait_for_server()

# #         return self._action_client_rotate_angle.send_goal_async(goal_msg)
# #     def angle_detection(self, data):
# #         r1 = data.find('(')
# #         r2 = data.find(')')
# #         newdata = data[r1+1:r2]
# #         print(newdata)
# #         self.send_goal(newdata)

# #     def get_data(self):
# #         data = self.tcp.read()

# def main(args = None):
#     # rclpy.init(args=args)
#     # rotate = Rotate()

#     # continuosly receive data
#     while True:
#         # rotate.get_data()
#         con = TCPserver(CREATE_IP, CREATE_PORT)
#         con.write('ab')
#         data = con.read()
        
#         print("Data received: "+ data)
#         time.sleep(1)
#         # try:
#         #     rclpy.spin(rotate)
#         # except KeyboardInterrupt:
#         #     print('\nCaught Keyboard Interrupt.')
#         # finally:
#         #     print('Done')
#         #     rclpy.shutdown()
    
# if __name__ == '__main__':
#     main()
'''
tcp_node.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 
This script is the base file for running a tcp server to communicate via the serial port on the Create®3.
'''

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

'''
import neccessary messages and modules
'''
from std_msgs.msg import String

import socket
import time

CREATE_IP = "192.168.1.162" 
CREATE_PORT = 8883

class TCPserver():
    '''
    this class creates the TCP server that will read serial information in from a port on the robot
    '''
    def __init__(self,IP,PORT):
        '''
        first we need to create a socket that the robot can connect to
        '''
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print('Failed to create socket')
            sys.exit()

        print('Socket Created')

        '''
        Connect the socket object to the robot using IP address (string) and port (int)
        '''
        self.client.connect((IP,PORT))
        print('Socket Connected to ' + CREATE_IP)
	
    def read(self):
        '''
        Read the response sent by robot upon connecting. This message will be the serial data sent in by
        what is connected to the port. 
        '''
        try:
            msg = self.client.recv(1024).decode('utf-8')
        except socket.error:
            print('Failed to read data')
        return msg

    def write(self,string):
        '''
        we can write serial data to the port as well using this function
        '''
        try:
            self.client.send(bytes(string.encode()))
        except socket.error:
            print('Failed to send data')

    def close(self):
        '''
        this function will close the socket when we are done with it
        '''
        self.client.close()


class TalkingTCP(Node):
    '''
    The TalkingTCP class is created which is a subclass of Node.
    This defines the class' constructor.
    '''
    def __init__(self):
        super().__init__('uartComm')
        
	
	# Here we initialize a timer, a publisher that will publish a string to the /uart topic, and the prior tcp class for easier use
	
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.uart_publisher = self.create_publisher(String,'/uart', 10)

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)

    def timer_callback(self):
        '''Once every second this function will be called. It reads in the serial data from the Create®3. 
Formats is properly and then publishes it to the /uart topic. If there is no data to publish, we will send the message nothing to publish'''
        data = self.tcp.read()  
        print(data)
        # if len(data) != 0:
        #     msg = String()
        #     msg.data = data 
        #     self.uart_publisher.publish(msg)
        # else: 
        #     msg = String()
        #     msg.data = 'nothing to publish'
        #     self.uart_publisher.publish(msg)


def main(args=None):
	
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
	
    '''
    The node is created and can be used in other parts of the script.
    '''
    uartComm = TalkingTCP()
    try:
        '''
        The node is "spun" so the functions can be executed. 
        '''
        rclpy.spin(uartComm)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        '''
        finally the node is shut down when we are done running the script
        '''
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()