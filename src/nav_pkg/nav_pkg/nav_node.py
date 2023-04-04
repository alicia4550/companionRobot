#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Twist
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from irobot_create_msgs.action import NavigateToPosition
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry

import time
import math


class NavNode(Node):
    def __init__(self, x, y):
        super().__init__('nav_node')

        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel', 10)
        self.front_speed = 0.0
        self.rotate_speed = 0.0
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.set_speed)
        
        self.subscription_ir = self.create_subscription(IrIntensityVector, '/ir_intensity', self.listener_callback_ir, qos_profile_sensor_data)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile_sensor_data)
        self.subscription_hazard = self.create_subscription(HazardDetectionVector, '/hazard_detection', self.listener_callback_hazard, qos_profile_sensor_data)

        self._action_client_navigate_to_position = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        self.intensity = [0] * 7

        self.x = float(x)
        self.y = float(y)

        self.hasReachedDestination = False
        self.isWallFollowing = False
        self.shouldTurnAfterBump = False

        self.goal_handle_navigate_to_position = None

        self.send_goal_navigate_to_position()                
    
    def listener_callback_ir(self, msg):
        readings = msg.readings
        for i in range(len(readings)):
            self.intensity[i] = readings[i].value

        ## Print IR Intensity values in the terminal
        # if not self.hasReachedDestination:
        #     self.get_logger().info(str(self.intensity))

        if not self.hasReachedDestination:
            ## Robot should rotate away from a corner if bumped 
            # self.shouldTurnAfterBump was set in the Hazard Detection subscription topic
            if self.shouldTurnAfterBump:
                if self.should_end_wall_fallow():
                    self.shouldTurnAfterBump = False
                else:
                    self.get_logger().info("Rotating")
                    self.front_speed = 0.5
                    self.rotate_speed = -0.1
            ## Rotate the robot so that its left side is parallel(ish) to the wall enough for it to move forward
            elif self.should_rotate():
                self.isWallFollowing = True
                if self.goal_handle_navigate_to_position is not None:
                    self.get_logger().info("Cancelling navigate to position")
                    self.cancel_goal_navigate_to_position()
                self.get_logger().info("Rotating")
                self.front_speed = 0.0
                self.rotate_speed = -0.1
            ## Move the robot forward
            # Is run when moving along a wall
            elif self.isWallFollowing and not self.should_end_wall_fallow():
                if self.goal_handle_navigate_to_position is not None:
                    self.get_logger().info("Cancelling navigate to position")
                    self.cancel_goal_navigate_to_position()
                self.get_logger().info("Forward")
                self.front_speed = 0.5
                self.rotate_speed = 0.0
            ## Is run after robot reaches the end of the wall after following
            # Robot will move forward a little bit to avoid running into the same obstacle when navigating to position
            elif self.isWallFollowing and self.should_end_wall_fallow():
                time.sleep(2)
                self.front_speed = 0.0
                self.rotate_speed = 0.0
                self.isWallFollowing = False
            ## Call the Navigate to Position action is no further obstacles are present
            else:
                if self.goal_handle_navigate_to_position is None and not self.isWallFollowing:
                    self.send_goal_navigate_to_position()


    def listener_callback_odom(self, msg):
        position = msg.pose.pose.position

        ## Print position values in the terminal
        if not self.hasReachedDestination:
            self.get_logger().info(str(position))

        ## Check if the destination has been reached
        if not self.hasReachedDestination and math.sqrt((self.x - position.x)**2 + (self.y - position.y)**2) < 0.1:
            self.get_logger().info("Has reached destination")
            self.front_speed = 0.0
            self.rotate_speed = 0.0
            self.hasReachedDestination = True

    def listener_callback_hazard(self, msg:HazardDetectionVector):
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                self.get_logger().warning(det)
                if self.goal_handle_navigate_to_position is not None:
                    self.get_logger().info("Cancelling navigate to position")
                    self.cancel_goal_navigate_to_position()
                self.shouldTurnAfterBump = True
    
    def should_move_forward(self):
        return all(sensor_val < 50 for sensor_val in self.intensity[-5:]) and self.intensity[1] < 100
    
    def should_rotate(self):
        return not all(sensor_val < 150 for sensor_val in self.intensity[-6:]) or (self.intensity[0] < self.intensity[1] and self.intensity[1] > 150)

    def should_end_wall_fallow(self):
        return all(sensor_val < 50 for sensor_val in self.intensity)
    
    def set_speed(self):
        velocity = Twist()
        velocity.linear.x = self.front_speed
        velocity.angular.z = self.rotate_speed
        self.publisher_cmd_vel.publish(velocity)
    
    def send_goal_navigate_to_position(self):
        self.get_logger().info("Send goal navigate to position")

        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = False
        goal_msg.max_translation_speed = 0.1

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0

        poseStamped = PoseStamped()
        poseStamped.pose = pose

        goal_msg.goal_pose = poseStamped

        self._action_client_navigate_to_position.wait_for_server()

        self.future_navigate_to_position = self._action_client_navigate_to_position.send_goal_async(goal_msg)
        self.future_navigate_to_position.add_done_callback(self.goal_response_callback_navigate_to_position)

    def goal_response_callback_navigate_to_position(self, future):
        self.get_logger().info("Goal response navigate to position")
        self.goal_handle_navigate_to_position = future.result()
    
    def cancel_goal_navigate_to_position(self):
        future = self.goal_handle_navigate_to_position.cancel_goal_async()
        future.add_done_callback(self.cancel_response_callback_navigate_to_position)

    def cancel_response_callback_navigate_to_position(self, future):
        self.get_logger().info("Cancelled navigate to position")
        self.goal_handle_navigate_to_position = None


def main(args=None):
    rclpy.init(args=args)

    nav_node = NavNode(-2,0)

    try:
        rclpy.spin(nav_node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()