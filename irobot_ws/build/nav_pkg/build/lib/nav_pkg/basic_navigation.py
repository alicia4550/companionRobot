#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseStamped
from irobot_create_msgs.action import RotateAngle, WallFollow, DriveDistance
import math
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
from rclpy.qos import qos_profile_sensor_data

from nodes.drive_distance_node import DriveDistanceNode
from nodes.get_odom_node import GetOdomNode


def get_delta_angle(x, y, get_odom_node):
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
    x, y, z = euler_from_quaternion(get_odom_node.orientation_)
    current_angle = z
    delta_angle = new_angle - current_angle

    # print("Current angle: " + str(current_angle))
    # print("Goal angle: " + str(new_angle))
    # print("Delta angle: " + str(delta_angle))

    return delta_angle

def euler_from_quaternion(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z

def hasReachedDestination(x, y, get_odom_node):
    delta_x = x - get_odom_node.position_.x
    delta_y = y - get_odom_node.position_.y
    return delta_x + delta_y < 0.25

def rotate(angle):
    rotate_angle_node = RotateAngleNode()
    future = rotate_angle_node.send_goal(angle)
    rclpy.spin_until_future_complete(rotate_angle_node, future)

def boundary_follow(set_speed_node, get_ir_node):
    rotate_node = RotateNode(0.01)
    while True:
        rclpy.spin_once(rotate_node)
        rclpy.spin_once(get_ir_node)

        if get_ir_node.intensity_[1] < 50 and get_ir_node.intensity_[0] > 100:
            break

    while True:
        rclpy.spin_once(set_speed_node)
        rclpy.spin_once(get_ir_node)

        if get_ir_node.intensity_[0] < 50:
            break

    drive_distance_node = DriveDistanceNode()
    future = drive_distance_node.send_goal()
    rclpy.spin_until_future_complete(drive_distance_node, future)

def obstacle(sensors):
    return not all(sensor_val < 250 for sensor_val in sensors)


def main(args=None):
    rclpy.init(args=args)

    get_odom_node = nodes.GetOdomNode()
    rclpy.spin_once(get_odom_node)

    x = 0.0
    y = 1.0

    delta_angle = get_delta_angle(x, y,get_odom_node)
    rotate(delta_angle)

    set_speed_node = SetSpeedNode(1.0)

    get_ir_node = GetIrNode()

    while True:
        rclpy.spin_once(get_odom_node)
        if hasReachedDestination(x, y, get_odom_node):
            print("hasReachedDestination")
            break
        
        rclpy.spin_once(get_ir_node)
        if obstacle(get_ir_node.intensity_):
            boundary_follow(set_speed_node, get_ir_node)
            delta_angle = get_delta_angle(x, y,get_odom_node)
            rotate(delta_angle)
        rclpy.spin_once(set_speed_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()