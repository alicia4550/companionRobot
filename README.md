
# companionRobot
This project runs on the iRobot Create3, with code in Python using ROS2.

## Requirements
1. [Ubuntu 22.04 (Jammy)](https://ubuntu.com/download/desktop)
2. [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
3. Colcon
	```
	sudo apt install python3-colcon-common-extensions
	```

## Run the project
1. Connect robot to Wifi (Phase 3 of the [iRobot Create3 Set Up Guide](https://edu.irobot.com/create3-setup)).
2. Open a new terminal window.
3. Navigate to the root directory.
4. Set up the terminal/source the required files:
	```
	source /opt/ros/humble/setup.bash
	source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
	source install/local_setup.bash
	```
	Note: you can also put these lines in your `.bashrc` file so you don't have to enter these lines every time you open a new terminal.
5. Build the project:
	```
	colcon build
	source install/local_setup.bash
	```
6. Run a package, replacing `<PACKAGE_NAME>` with the name of the package you want to run (packages seen in Package layout below):
	```
	ros2 launch <PACKAGE_NAME> _launch.py
	```

## Package layout
* `nav_pkg`: Package used to make the robot navigate to a given point
* `roam_pkg`: Package used to make the robot roam around the room
* `serial_pkg`: Package used for serial communication between the robot and the Bluefruit module
