#!/usr/bin/env python3
import rclpy
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.counter_ = 0
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.navigate_to)

        # pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

        # self.subscription_battery_state = self.create_subscription(
        #     BatteryState,
        #     '/battery_state',
        #     self.get_battery_state,
        #     qos_profile)

    def navigate_to(self):
        self.get_logger().info("Hello")
        if (self.counter_ % 2 == 0):
            # velocity = self.turn_left
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = math.pi
            self.publisher_cmd_vel.publish(velocity)
        else:
            # velocity = self.move_ahead
            velocity = Twist()
            velocity.linear.x = 0.3
            velocity.angular.z = 0.0
            self.publisher_cmd_vel.publish(velocity)
        self.counter_ += 1
        # self.get_logger().info("Hello " + str(self.counter_))
    
    def turn_left():
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.3
        return velocity
    
    def move_ahead():
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0
        return velocity
 
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()