#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseStamped
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
        self.set_speed()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.set_speed)

    def set_speed(self):

        velocity = Twist()
        velocity.linear.x = self.speed_
        velocity.angular.z = 0.0
        self.publisher_cmd_vel.publish(velocity)

class GetOdomNode(Node):
    def __init__(self):
        super().__init__('get_odom_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.position_callback, qos_profile_sensor_data)
        self.position_ = Point()
        self.orientation_ = Quaternion()

    def position_callback(self, msg):
        self.position_ = msg.pose.pose.position
        self.orientation_ = msg.pose.pose.orientation
        self.get_logger().info("Current angle: x={0:.2f}, y={1:.2f}, z={2:.2f}".format(self.orientation_.x, self.orientation_.y, self.orientation_.z))
        self.get_logger().info("Current position: x={0:.2f}, y={1:.2f}".format(self.position_.x, self.position_.y))

class ResetPose(Node):
    def __init__(self):
        super().__init__("reset_pose_node")
        self.publisher_initial_pose = self.create_publisher(
            PoseStamped,
            '/initialpose',
            10
        )
        self.reset_pose()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.reset_pose)

    def reset_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.publisher_initial_pose.publish(pose)

def get_delta_angle(x, y, get_odom_node):
    # Calculate angle of vector pointing to destination - relative to 0 degrees
    print("Current angle: x={0:.2f}, y={1:.2f}, z={2:.2f}".format(get_odom_node.orientation_.x, get_odom_node.orientation_.y, get_odom_node.orientation_.z))
    delta_x = x - get_odom_node.position_.x
    delta_y = y - get_odom_node.position_.y

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
    delta_angle = get_odom_node.orientation_.z - new_angle

    print("Goal angle: " + str(new_angle))
    print("Delta angle: " + str(delta_angle))

    return delta_angle

def hasReachedDestination(x, y, get_odom_node):
    delta_x = x - get_odom_node.position_.x
    delta_y = y - get_odom_node.position_.y
    return delta_x + delta_y < 0.5

def rotate(angle):
    rotate_angle_node = RotateAngleNode()
    future = rotate_angle_node.send_goal(angle)
    rclpy.spin_until_future_complete(rotate_angle_node, future)


def main(args=None):
    rclpy.init(args=args)
    
    reset_pose_node = ResetPose()
    rclpy.spin_once(reset_pose_node)

    get_odom_node = GetOdomNode()
    rclpy.spin_once(get_odom_node)
    delta_angle = get_delta_angle(0.5,0.5,get_odom_node)
    rotate(delta_angle)
    rclpy.spin_once(get_odom_node)
    print("Current angle: x={0:.2f}, y={1:.2f}, z={2:.2f}".format(get_odom_node.orientation_.x, get_odom_node.orientation_.y, get_odom_node.orientation_.z))

    set_speed_node = SetSpeedNode(2.0)

    while True:
        rclpy.spin_once(get_odom_node)
        if hasReachedDestination(0.5, 0.5, get_odom_node):
            print("hasReachedDestination")
            break
        rclpy.spin_once(set_speed_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()