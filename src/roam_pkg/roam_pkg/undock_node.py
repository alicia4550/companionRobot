# this file undocks the robot if it is docked
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from irobot_create_msgs.msg import DockStatus
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock


# this node subscribes to the /dock_status topic and is an action client that sends undock goal request to action server
class UndockNode(Node):
    def __init__(self):
        super().__init__('undock_node') #node name

        print('Creating subscription to to the DockStatus type over the /dock_status topic')
        # subscribes to type DockStatus over /dock_status topic
        self.subscription = self.create_subscription(DockStatus, '/dock_status', self.listener_callback, qos_profile_sensor_data)

        print('Initiating a new Undock action server...')
         # initialize a action server and include Undock as type of action and action name /undock
        self._action_client_undock = ActionClient(self, Undock, '/undock')
    
    # The subscriber's callback listens and as soon as it receives the message, this function runs. 
    # This callback function is basically printing what it hears.
    def listener_callback(self, msg:DockStatus):
        dock_status = msg.is_docked
        if (dock_status): # if the robot is docked
            print("Robot is docked.")
            self.send_goal() # undock action
        else:
            print("Robot is already undocked.") # if the robot is undocked, do nothing.        

    def send_goal(self):
        goal_msg = Undock.Goal()
        print('Goal Message: ' + str(goal_msg))
        
        # wait for the action server to be available
        self._action_client_undock.wait_for_server()

        return self._action_client_undock.send_goal_async(goal_msg)




def main(args=None):
    # initialize rclpy libray
    rclpy.init(args=args)
    # create node
    undock_node = UndockNode()
    
    try: 
        print("Spin node")  
        future = undock_node.send_goal()
        rclpy.spin_until_future_complete(undock_node,future) 
        
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("Undocked.")
        undock_node.destroy_node()
        rclpy.try_shutdown()

    
    
if __name__ == '__main__':
    main()