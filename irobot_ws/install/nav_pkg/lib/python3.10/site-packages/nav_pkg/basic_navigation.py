#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point
from irobot_create_msgs.action import RotateAngle
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage
from time import sleep
from nav_msgs.msg import Odometry
import threading
from rclpy.qos import qos_profile_sensor_data

class RotateAngleNode(Node):
    def __init__(self):
        super().__init__("rotate_angle_node")
        self._action_client_rotate_angle = ActionClient(self, RotateAngle, 'rotate_angle')

    def send_goal(self, angle):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle

        self._action_client_rotate_angle.wait_for_server()

        return self._action_client_rotate_angle.send_goal_async(goal_msg)
    
class SetSpeedNode(Node):
    def __init__(self, speed):
        super().__init__("set_speed_node")
        self.speed_ = speed
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_odom_node = GetOdomNode()
        self.set_speed()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.set_speed)

    def set_speed(self):

        velocity = Twist()
        velocity.linear.x = self.speed_
        velocity.angular.z = 0.0
        self.publisher_cmd_vel.publish(velocity)

class GetTransformNode(Node):
    def __init__(self):
        super().__init__("get_transform_node")
        self.subscription_tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.get_tf,
            10)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.get_tf)

        self.translation_ = Vector3()
        self.rotation_ = Quaternion()

    def get_tf(self, msg: TFMessage):
        # self.get_logger().info(msg.transforms)
        self.translation_ = msg.transforms[1].transform.translation
        self.rotation_ = msg.transforms[1].transform.rotation
        # print(self.translation_)
        self.get_logger().info(self.translation_.x)

class GetOdomNode(Node):
    def __init__(self):
        super().__init__('get_odom_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.position_callback, qos_profile_sensor_data)
        self.position_ = Point()
        self.orientation_ = Quaternion()

    def position_callback(self, msg):
        self.position_ = msg.pose.pose.position
        self.orientation_ = msg.pose.pose.orientation
        self.get_logger().info("Current position: x={0:.2f}, y={1:.2f}".format(self.position_.x, self.position_.y))

class NavigateNode(Node):
    def __init__(self, x, y, speed):
        super().__init__("navigate_node")
        self.x_ = x
        self.y_ = y
        self.speed_ = speed

        self.get_odom_node = GetOdomNode()
        # rclpy.spin(self.get_odom_node)

        t1 = threading.Thread(target=self.spin_odom_node)
        t2 = threading.Thread(target=self.navigate)

        t1.start()
        t2.start()

        t1.join()
        t2.join()

        # self.navigate()

    def spin_odom_node(self):
        rclpy.spin(self.get_odom_node)

    def get_delta_angle(self):
        # Calculate angle of vector pointing to destination - relative to 0 degrees
        delta_x = self.x_ - self.get_odom_node.position_.x
        delta_y = self.y_ - self.get_odom_node.position_.y

        if delta_x == 0 and delta_y >= 0:
            new_angle = math.pi/2
        elif delta_x == 0 and delta_y < 0:
            new_angle = math.pi*1.5
        else:
            new_angle = math.atan(abs(delta_y)/abs(delta_x))

        # Trig Rules
        if delta_x < 0 and delta_y > 0:
            new_angle = math.pi - new_angle
        elif delta_x < 0 and delta_y < 0:
            new_angle = math.pi + new_angle
        elif delta_x > 0 and delta_y < 0:
            new_angle = 2*math.pi - new_angle

        # Get difference between destination angle and current heading of robot
        delta_angle = self.get_odom_node.orientation_.z - new_angle

        # Get positive angle value
        if delta_angle < 0:
            delta_angle = 2*math.pi + delta_angle

        return delta_angle

    def navigate(self):
        delta_angle = self.get_delta_angle()

        self.rotate_angle_node = RotateAngleNode()
        # self.rotate_angle_node.send_goal(delta_angle)
        future = self.rotate_angle_node.send_goal(delta_angle)
        rclpy.spin_until_future_complete(self.rotate_angle_node, future)
        # sleep(5)

        self.set_speed_node = SetSpeedNode(self.speed_)
        rclpy.spin(self.set_speed_node)

        print("done spinning")
        

    def stop_if_destination_reached(self):
        while True:
            # # rclpy.shutdown(self.get_tf_node)
            # self.get_tf_node = GetTransformNode()
            if (self.hasReachedDestination()):
                self.set_speed_node.destroy_node()
                self.set_speed_node = SetSpeedNode(0.0)
                break

    def hasReachedDestination(self):
        print(str(self.get_odom_node.position_.x) + " " + str(self.get_odom_node.position_.y))
        delta_x = self.x_ - self.get_odom_node.position_.x
        delta_y = self.y_ - self.get_odom_node.position_.y
        if (delta_x + delta_y < 5):
            return True
        else:
            return False

def rotate(angle, args=None):
    rclpy.init(args=args)
    rotate_angle_node = RotateAngleNode()
    future = rotate_angle_node.send_goal(angle)
    rclpy.spin_until_future_complete(rotate_angle_node, future)
    rclpy.shutdown()

def forward(speed, args=None):
    rclpy.init(args=args)
    set_speed_node = SetSpeedNode(speed)
    rclpy.spin(set_speed_node)
    rclpy.shutdown()

def get_tf(args=None):
    rclpy.init(args=args)
    get_tf_node = GetTransformNode()
    rclpy.spin(get_tf_node)
    rclpy.shutdown()

def navigate_to(x, y, speed, args=None):
    rclpy.init(args=args)
    navigate_node = NavigateNode(x,y,speed)
    rclpy.spin(navigate_node)
    rclpy.shutdown()
    

def main(args=None):
    # rotate(math.pi/2)
    # forward(2.0)
    # get_tf()
    navigate_to(10,10,2.0)
    # rclpy.init(args=args)

    # get_odom_node = GetOdomNode()

    # rclpy.spin(get_odom_node)

    # get_odom_node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()