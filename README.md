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
3. Navigate to the `irobot_ws` directory.
4. Set up the terminal/source the required files:
	```
	source /opt/ros/humble/setup.bash
	source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
	source install/local_setup.bash
	```
5. Build the project:
	```
	colcon build
	```
6. Launch the project:
	```
	ros2 launch nav_pkg _launch.py