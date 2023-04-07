# this file undocks the robot if it is docked
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from irobot_create_msgs.msg import DockStatus
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState


# this node subscribes to the battery state
class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node') #node name

        self.subscription_battery = self.create_subscription(BatteryState, '/battery_state', self.listener_callback_battery, qos_profile_sensor_data)
        self.battery_percentage = 100
    
    def listener_callback_battery(self, msg):
        self.battery_percentage = msg.percentage * 100
        self.get_logger().info("Battery Percentage: " + str(self.battery_percentage) + "%")


def main(args=None):
    # initialize rclpy libray
    rclpy.init(args=args)
    # create node
    dock_node = BatteryNode()
    
    try: 
        print("Spin node")
        rclpy.spin(dock_node)        
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        dock_node.destroy_node()
        rclpy.try_shutdown()

    
    
if __name__ == '__main__':
    main()