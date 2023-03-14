#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import NavigateToPosition, WallFollow, DriveDistance
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from action_msgs.msg._goal_status import GoalStatus

from threading import Lock
import time


class NavNode(Node):
    def __init__(self, x, y):
        super().__init__('nav_node')
        self.lock = Lock()
        
        self.subscription_ir = self.create_subscription(IrIntensityVector, '/ir_intensity', self.listener_callback_ir, qos_profile_sensor_data)

        self._action_client_navigate_to_position = ActionClient(self, NavigateToPosition, '/navigate_to_position')
        self._action_client_wall_follow = ActionClient(self, WallFollow, '/wall_follow')
        self._action_client_drive_distance = ActionClient(self, DriveDistance, '/drive_distance')

        self.intensity = [0] * 7

        self.x = float(x)
        self.y = float(y)

        self.goal_handle_navigate_to_position = None
        self.goal_handle_wall_follow = None
        self.goal_handle_drive_distance = None

        self.send_goal_navigate_to_position()
    
    def listener_callback_ir(self, msg):
        readings = msg.readings
        for i in range(len(readings)):
            self.intensity[i] = readings[i].value
        
        if self.obstacle(self.intensity):
            if self.goal_handle_wall_follow is not None:
                if self.should_stop_wall_follow(self.intensity):
                    with self.lock:         
                        self.cancel_goal_wall_follow()
                        time.sleep(0.5)
                        self.send_goal_drive_distance()
                        time.sleep(5)
                        self.send_goal_navigate_to_position()
                        self.goal_handle_wall_follow = None
            else:
                self.cancel_goal_navigate_to_position()
                self.send_goal_wall_follow()

    def obstacle(self, sensors):
        return not all(sensor_val < 200 for sensor_val in sensors)
    
    def should_stop_wall_follow(self, sensors):
        return all(sensor_val == 15 for sensor_val in sensors[:6]) and sensors[6] > 600

    def send_goal_navigate_to_position(self):
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = False

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
        self.goal_handle_navigate_to_position = future.result()
    
    def cancel_goal_navigate_to_position(self):
        self.goal_handle_navigate_to_position.cancel_goal_async()

    def send_goal_wall_follow(self):
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = 1
        goal_msg.max_runtime = Duration(sec = 60, nanosec = 0)

        self._action_client_wall_follow.wait_for_server()

        self.future_wall_follow = self._action_client_wall_follow.send_goal_async(goal_msg)
        self.future_wall_follow.add_done_callback(self.goal_response_callback_wall_follow)

    def goal_response_callback_wall_follow(self, future):
        self.goal_handle_wall_follow = future.result()
    
    def cancel_goal_wall_follow(self):
        self.goal_handle_wall_follow.cancel_goal_async()

    def send_goal_drive_distance(self):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.5
        goal_msg.max_translation_speed = 0.15

        self._action_client_drive_distance.wait_for_server()

        self.future_drive_distance = self._action_client_drive_distance.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    nav_node = NavNode(-2, 0)

    try:
        rclpy.spin(nav_node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        nav_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()