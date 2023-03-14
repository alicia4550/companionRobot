#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# import rospy

import math
import time

from .drive_distance_node import DriveDistanceNode
from .get_ir_node import GetIrNode
from .get_odom_node import GetOdomNode
from .rotate_angle_node import RotateAngleNode
from .rotate_node import RotateNode
from .set_speed_node import SetSpeedNode


def getDeltaAngle(x, y, get_odom_node):
    # Calculate angle of vector pointing to destination - relative to 0 degrees
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
    x, y, z = get_odom_node.euler_from_quaternion()
    current_angle = z
    delta_angle = new_angle - current_angle

    return delta_angle

def hasReachedDestination(x, y, get_odom_node):
    delta_x = abs(x - get_odom_node.position_.x)
    delta_y = abs(y - get_odom_node.position_.y)
    return delta_x + delta_y < 0.25

def rotate(angle):
    rotate_angle_node = RotateAngleNode()
    future = rotate_angle_node.send_goal(angle)
    rclpy.spin_until_future_complete(rotate_angle_node, future)

def boundaryFollow(set_speed_node, get_ir_node):
    rotate_node = RotateNode(0.01)
    # rotate_node = RotateNode(0.1)
    while True:
        rclpy.spin_once(rotate_node)
        rclpy.spin_once(get_ir_node)
        
        if (get_ir_node.intensity_[1] < 50 and get_ir_node.intensity_[0] > 100):
            break

    while True:
        rclpy.spin_once(set_speed_node)
        rclpy.spin_once(get_ir_node)

        if get_ir_node.intensity_[0] < 50:
            break
        if obstacle(get_ir_node.intensity_):
            boundaryFollow(set_speed_node, get_ir_node)

def obstacle(sensors):
    return not all(sensor_val < 250 for sensor_val in sensors)


def main(args=None):
    rclpy.init(args=args)

    get_odom_node = GetOdomNode()
    rclpy.spin_once(get_odom_node)

    x = 3.0
    y = 0.0

    delta_angle = getDeltaAngle(x, y,get_odom_node)
    rotate(delta_angle)

    set_speed_node = SetSpeedNode(1.0)

    get_ir_node = GetIrNode()

    rclpy.logging.get_logger("get_odom_node").info("ASDF")

    while True:
        rclpy.spin_once(get_odom_node)
        if hasReachedDestination(x, y, get_odom_node):
            rclpy.logging.get_logger("get_odom_node").info("hasReachedDestination")
            break
        
        rclpy.spin_once(get_ir_node)
        if obstacle(get_ir_node.intensity_):
            boundaryFollow(set_speed_node, get_ir_node)

            set_speed_node.set_speed()
            time.sleep(0.5)
            set_speed_node.stop()

            rclpy.spin_once(get_odom_node)
            delta_angle = getDeltaAngle(x, y,get_odom_node)
            rotate(delta_angle)
        rclpy.spin_once(set_speed_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()