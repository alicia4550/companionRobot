
# companionRobot
This project runs on the iRobot Create3, with code in Python using ROS2.

## Requirements
* [Ubuntu 22.04 (Jammy)](https://ubuntu.com/download/desktop)
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* Colcon
	```
	sudo apt install python3-colcon-common-extensions

## Setup
1. Connect robot to Wifi (Phase 3 of the [iRobot Create3 Set Up Guide](https://edu.irobot.com/create3-setup)).
	
	> Take note of the robot's MAC address which can be found in the About tab.
2. Connect the ESP32 to the robot's adapter board using a Micro-B to USB-C cable.
3. Open the robot's top plate. 
4. Toggle the switch on the adapter board to USB mode. You should only see an orange light now. 
5. Close the robot's top plate.
6. Open a new terminal window.
7. Find the robot's IP address. You can scan all IP addresses (arp -a in Windows) in terminal and match the robot's MAC address with the corresponding IP address.
8. Open a browser tab and navigate to the robot's IP address.
9. In the Log tab of the web server, copy the name of the tty device connected to the robot (e.g. ttyUSB0).
10. Go to the Serial Port Foward page in the Beta Features tab. Enter the following information, then click Save.
	```
	TTY device to forward: (paste the name of the tty device in step 5)
	Baudrate: 115200
	External port number: 8883
	```
11. Power cycle the robot.
12. Open a new terminal window.
13. Navigate to the root directory.
14. Edit the robot's IP address in `/src/serial_pkg/serial_pkg/main.py`. 
15. Set up the terminal or source the required files:
	```
	source /opt/ros/humble/setup.bash
	source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
	source install/local_setup.bash
	```
	Note: you can also put these lines in your `.bashrc` file so you don't have to enter these lines every time you open a new terminal.

16. Build the project:
	```
	colcon build
	source install/local_setup.bash
	```
## Run the project
1. Run a package, replacing `<PACKAGE_NAME>` with the name of the package you want to run (packages seen in Package layout below):
	```
	ros2 launch <PACKAGE_NAME> _launch.py
	```
## Package layout
* `nav_pkg`: Package used to make the robot navigate to a given point
* `roam_pkg`: Package used to make the robot roam around the room
* `serial_pkg`: Package used for serial communication between the robot and the ESP32
* `controller_pkg`: Package used for robot startup and switching between different operating modes


