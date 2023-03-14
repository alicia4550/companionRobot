#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from irobot_create_msgs.msg import HazardDetectionVector

from rclpy.action import ActionClient
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration

from irobot_create_msgs.action import RotateAngle



class RoamNode(Node):
    def __init__(self, speed):
        super().__init__("roam_node")

        # Set speed publisher
        self.speed_ = speed
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel', 10)
        self.set_speed()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.set_speed)

        # Wall follow action
        self._action_client_wall_follow = ActionClient(self, WallFollow, 'wall_follow')

        # Rotate action
        self._action_client_rotate = ActionClient(self, RotateAngle, '/rotate_angle')

        # IR detection subscriber
        print('Creating subscription to to the IrIntensityVector type over the /ir_intensity topic')
        # subscribes to type IrIntensityVector over /ir_intensity topic
        self.subscription_ir = self.create_subscription(IrIntensityVector, '/ir_intensity', self.listener_callback_ir, qos_profile_sensor_data)
        self.intensity_ = [0] * 7

        # Hazard detection subscriber
        self.subscription_hazard = self.create_subscription(HazardDetectionVector, '/hazard_detection', self.listener_callback_hazard, qos_profile_sensor_data)

    # This publishes speed to the /cmd_vel topic to set robot speed
    def set_speed(self):
        velocity = Twist()
        velocity.linear.x = self.speed_
        velocity.angular.z = 0.0
        self.publisher_cmd_vel.publish(velocity)
    
    # This sends an action request to wall follow
    def send_goal_wall_follow(self, side):
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = side # 1 is counterclockwise, -1 is clockwise
        goal_msg.max_runtime = Duration(sec = 1, nanosec = 500000000) # run for 1s

        self._action_client_wall_follow.wait_for_server()

        return self._action_client_wall_follow.send_goal_async(goal_msg)
    
    # This sends an action request to rotate at an angle
    def send_goal_rotate(self, angle, max_rotation_speed):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client_rotate.wait_for_server()

        return self._action_client_rotate.send_goal_async(goal_msg)

    # The subscriber's callback listens and as soon as it receives the message, this function runs. 
    # This callback function is basically printing what it hears. 
    def listener_callback_ir(self, msg:IrIntensityVector):
        print('Now listening to IR sensor readings...')
        print('Printing IR sensor readings:')
        i = 0
        for reading in msg.readings:
            val = reading.value
            # name = reading.header.frame_id
            self.intensity_[i] = val
            i += 1
            #print(name + ": " + str(val))
        print(self.intensity_)

        
        # if any IR sensor detects an obstacle within a desired range(intensity)
        if self.obstacle(self.intensity_):
            self.speed_ = 0.1
            # if facing a wall, all sensor values greater than 0
            if all(sensor > 0 for sensor in self.intensity_):
                # wall follow to the left
                self.send_goal_wall_follow(-1)
            
            else:
                print("Obstacles detected! Slow down.")
                # if the obstacle is straight ahead
                if self.intensity_[3] == max(self.intensity_) or self.intensity_[4] == max(self.intensity_):
                    print("Obstacles ahead!")
                    # turn 90 degrees to the right
                    self.speed_ = 0.0
                    self.send_goal_rotate(angle=1.57, max_rotation_speed=0.5)
                # if the obstacle is on either side
                else:
                    left_ir_avg= sum(self.intensity_[:4])/4
                    right_ir_avg = sum(self.intensity_[-3:])/3

                    # if obstacle on the left is closer
                    if left_ir_avg > right_ir_avg:
                        print("Obstacles on left, avoid!")
                        # turn 45 degrees to right
                        self.speed_ = 0.0
                        self.send_goal_rotate(angle=0.785, max_rotation_speed=0.5)
                        
                    # if obstacle on the right is closer
                    elif right_ir_avg > left_ir_avg:
                        print("Obstacles on right, avoid!")
                        # turn 45 degrees to left
                        self.speed_ = 0.0
                        self.send_goal_rotate(angle=-0.785, max_rotation_speed=0.5)
                    
                    # obstacle on both sides are equally close (rare)
                    else: 
                        # stop for now 
                        print("Obstacles on both sides equally close!")
                        self.speed_ = 0.0

        # no major obstacles in the way
        else:
            print("Clear way ahead.")
            self.speed_ = 0.5

        print(self.intensity_)

    def obstacle(self, sensors):
        return not all(sensor_val < 200 for sensor_val in sensors)
    
    def listener_callback_hazard(self, msg:HazardDetectionVector):
        for detection in msg.detections:
            det = detection.header.frame_id

            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.speed_ = 0.0
                    # rotate left by 180 degrees
                    self.send_goal_rotate(angle=-3.14, max_rotation_speed=0.5) 
                elif det == "bump_left":
                    self.speed_ = 0.0
                    # rotate right by 180 degrees
                    self.send_goal_rotate(angle=3.14, max_rotation_speed=0.5)
                elif det == "bump_front_left":
                    self.speed_ = 0.0
                    # rotate right by 120 degrees
                    self.send_goal_rotate(angle=2.09, max_rotation_speed=0.5)
                elif det == "bump_front_right":
                    self.speed_ = 0.0
                    # rotate left by 120 degrees
                    self.send_goal_rotate(angle=-2.09, max_rotation_speed=0.5)
                elif det == "bump_front_center":
                    self.speed_ = 0.0
                    # rotate by 180 degrees
                    self.send_goal_rotate(angle=-3.14, max_rotation_speed=0.5)
            
                

def main(args=None):
    # initialize rclpy libray
    rclpy.init(args=args)
    # create node
    roam_node = RoamNode(0.5)
    
    # spin the node so its callbacks are called
    print('Callbacks are called.')
    try:
        rclpy.spin(roam_node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        roam_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()