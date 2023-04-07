'''This script is adapted from tcp_node.py of Tufts Create®3 Educational Robot Example by Maddie Pero 
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

CREATE_IP = "192.168.1.167" 
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


class TCPNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
	
	# Here we initialize a timer, a publisher that will publish a string to the /uart topic, and the prior tcp class for easier use

        self.tcp = TCPserver(CREATE_IP,CREATE_PORT)

        self.msg = ""

    def timer_callback(self):
        '''Once every second this function will be called. It reads in the serial data from the Create®3. 
Formats is properly and then publishes it to the /uart topic. If there is no data to publish, we will send the message nothing to publish'''
        data = self.tcp.read()  
        if len(data) != 0:
            msg = String()
            msg.data = data 
            self.mg = msg
        else: 
            msg = String()
            msg.data = ''
            self.mg = msg



def main(args=None):
	
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
	
    '''
    The node is created and can be used in other parts of the script.
    '''
    uartComm = TCPNode()
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