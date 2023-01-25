#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

import math

robot = Create3(Bluetooth())
speed = 5
th = 250

# Continuously print position of robot (x, y, heading)
@event(robot.when_play)
async def play(robot):
    while True:
        await print_pos(robot)

# Move robot
@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await move_to(robot, 0, 900) # coordinates in mm

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
    if (abs(new_x - pos.x) < 25 and abs(new_y - pos.y) < 25): # threshold value to determine if robot is within 1.5 cm of its destination
        return True
    else:
        return False

# Get angle for robot to turn
async def get_delta_angle(robot, new_x, new_y):
    pos = await robot.get_position()

    # Calculate angle of vector pointing to destination - relative to 0 degrees
    delta_x = new_x - pos.x
    delta_y = new_y - pos.y

    if delta_x == 0 and delta_y >= 0:
        new_angle = 90
    elif delta_x == 0 and delta_y < 0:
        new_angle = 270
    else:
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

    return delta_angle

# Navigate to specified position
async def move_to(robot, new_x, new_y):
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
        if obstacle(sensors):
            print("obstacle")
            await boundary_follow(robot)
            await move_to(robot, new_x, new_y)
            break

    await print_pos(robot)

# Check for obstacles
def obstacle(sensors):
    return not all(sensor_val < th for sensor_val in sensors)

# Follow obstacle boundary
async def boundary_follow(robot):
    finishedTurning = False
    while not finishedTurning:
        await robot.turn_right(5)
        sensors = (await robot.get_ir_proximity()).sensors
        print("turning:", sensors)
        if (sensors[1] < 50 and sensors[0] > 100):
            finishedTurning = True
            await robot.turn_right(5)
    await robot.set_wheel_speeds(speed,speed)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if (sensors[0] < 50):
            await robot.move(20)
            return

robot.play()