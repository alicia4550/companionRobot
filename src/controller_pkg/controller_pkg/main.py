#!/usr/bin/env python3

import rclpy
import time

from rclpy.executors import ExternalShutdownException


from controller_pkg.follow_node import FollowNode
from controller_pkg.nav_node import NavNode
from controller_pkg.roam_node import RoamNode
from controller_pkg.dock_node import DockNode  
from controller_pkg.undock_node import UndockNode
from controller_pkg.battery_node import BatteryNode
from controller_pkg.serial_node import TCPNode

def main(args=None):
    rclpy.init(args=args)
   
    dock = DockNode()
    undock = UndockNode()
    battery = BatteryNode()
    #serial = TCPNode()

    # start up
    
    battery_full = True
    serial = '2'  
    try:
        while True:
            rclpy.spin_once(battery) # subscribe to /battery_state
            #rclpy.spin(serial) # start serial communication with ESP32
            # undock first
            print("1")
            # future = undock.send_goal()
            # rclpy.spin_until_future_complete(undock,future)
            # set battery status
            if (battery.battery_percentage < 10):
                battery_full = False
            else:
                battery_full = True # battery is charged

            # Normal Operating Modes..exit normal operating mode if battery level is low.
            while (battery_full):
                # check for follow request
                if (len(serial) != 0):
                    print("Robot is in FOLLOW mode. Navigating...")
                    navigate= NavNode(2, 1)
                    rclpy.spin_once(navigate)
                    if (battery.battery_percentage < 10):
                        battery_full = False
                        navigate.destroy_node()
                    
                # follow the owner until follow request is terminated or if battery level is low
                while (len(serial) != 0):
                    print("Robot is in FOLLOW mode. Following...")
                    follow = FollowNode()
                    rclpy.spin_once(follow)
                    if (battery.battery_percentage < 10):
                        battery_full = False
                        follow.destroy_node() 
                        break
                   
                
                # cycle between ROAM and IDLE modes
                startTime = time.time()

                roam = RoamNode(0.5)
                # 10 seconds timer
                print("Robot is in ROAMING mode.")
                while (time.time() - startTime) < 10:
                    rclpy.spin_once(roam)

                roam.destroy_node()
                print("Robot is in IDLE mode.")

                time.sleep(10)
        
            rclpy.spin_once(dock)
            
            # stay docked if battery is not fully charged
            while (battery.battery_percentage < 100):
                time.sleep(0.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        undock.destroy_node()
        dock.destroy_node()
        battery.destroy_node()
        #serial.destroy_node()
        roam.destroy_node()
        navigate.destroy_node()
        follow.destroy_node()
        rclpy.try_shutdown()
            

    
    
       

if __name__ == "__main__":
    main()

