# this file undocks the robot if it is docked
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from irobot_create_msgs.msg import DockStatus
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock

from sensor_msgs.msg import BatteryState


# this node subscribes to the /dock_status topic and is an action client that sends undock goal request to action server
class DockNode(Node):
    def __init__(self):
        super().__init__('dock_node') #node name

        self.subscription_battery = self.create_subscription(BatteryState, '/battery_state', self.listener_callback_battery, qos_profile_sensor_data)

        print('Initiating a new Dock action server...')
         # initialize a action server and include Undock as type of action and action name /undock
        self._action_client_dock = ActionClient(self, Dock, '/dock')
    
    def listener_callback_battery(self, msg):
        battery_percentage = msg.percentage * 100
        self.get_logger().info("Battery Percentage: " + str(battery_percentage) + "%")

        if battery_percentage < 10:
            self.send_dock_goal()

    def send_dock_goal(self):
        goal_msg = Dock.Goal()
        print('Goal Message: ' + str(goal_msg))
        
        # wait for the action server to be available
        self._action_client_dock.wait_for_server()

        return self._action_client_dock.send_goal_async(goal_msg)



def main(args=None):
    # initialize rclpy libray
    rclpy.init(args=args)
    # create node
    dock_node = DockNode()
    
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