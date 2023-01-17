#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

import math

robot = Create3(Bluetooth())
speed = 2
th = 150

# Continuously print position of robot (x, y, heading)
@event(robot.when_play)
async def play(robot):
    while True:
        await print_pos(robot)
# Move robot
@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await move_to(robot, 500, 500) # cm x 10

# Return value with two decimal places
def f(value):
    return format(value, '.2f')

# Print current position of robot (x, y, heading)
async def print_pos(robot):
    pos = await robot.get_position()
    print('🐢 current(x  y  heading) = (', f(pos.x),  f(pos.y), f(pos.heading), ')')

# Check if robot has reached its destination - correct to 1.5cm
async def reachedDestination(robot, new_x, new_y):
    pos = await robot.get_position()
    if (abs(new_x - pos.x) < 15 and abs(new_y - pos.y) < 15): # threshold value to determine if robot is within 1.5 cm of its destination
        return True
    else:
        return False

# Get angle for robot to turn
async def get_delta_angle(robot, new_x, new_y):
    pos = await robot.get_position()

    # Calculate angle of vector pointing to destination - relative to 0 degrees
    delta_x = new_x - pos.x
    delta_y = new_y - pos.y
    new_angle = math.degrees(math.atan(abs(delta_y)/abs(delta_x)))

    # Trig Rules
    if delta_x < 0 and delta_y > 0:
        new_angle = 180 - new_angle
    elif delta_x < 0 and delta_y < 0:
        new_angle = 180 + new_angle
    elif delta_x > 0 and delta_y < 0:
        new_angle = 360 - new_angle

    # Get difference between destination angle and current heading of robot
    delta_angle = pos.heading - new_angle

    # Get positive angle value
    if delta_angle < 0:
        delta_angle = 360 + delta_angle

    print(delta_angle)
    return delta_angle

# Navigate to specified position
async def move_to(robot, new_x, new_y):
    # await print_pos(robot)
    delta_angle = await get_delta_angle(robot, new_x, new_y)

    # Turn right or left depending on the angle
    if delta_angle < 180:
        await robot.turn_right(delta_angle)
    else:
        await robot.turn_left(360-delta_angle)

    # Move robot forward
    await robot.set_wheel_speeds(speed, speed)

    hasReachedDestination = False

    while True:
        # Stop robot if destination has been reached
        hasReachedDestination = await reachedDestination(robot, new_x, new_y)
        if hasReachedDestination:
            await robot.set_wheel_speeds(0,0)
            break

        # Change direction if there is a front obstacle
        sensors = (await robot.get_ir_proximity()).sensors
        if front_obstacle(sensors):
            print("front obstacle")
            await backoff(robot)
            await move_to(robot, new_x, new_y)
            break

    await print_pos(robot)

# Check if there is an obstacle in front of the robot
def front_obstacle(sensors):
    return sensors[3] > th

# Backoff from a front obstacle - move back and change angle
async def backoff(robot):
    await robot.set_lights_rgb(255, 80, 0)
    await robot.move(-20)
    await robot.turn_left(45)
    await robot.move(50)

robot.play()