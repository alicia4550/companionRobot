from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())  # Will connect to the first robot found.
threshold = 100


def f(value):
    return format(value, '.2f')


async def print_pos(robot):
    pos = await robot.get_position()
    print('🐢 (x  y  heading) = (', f(pos.x),  f(pos.y), f(pos.heading), ')')


async def backoff(robot):
    await robot.move(-20)
    await robot.turn_left(45) # counterclockwise 45 degrees


def front_obstacle(sensors):
    print(sensors[3]) # centre front sensor
    return sensors[3] > threshold


@event(robot.when_play)
async def play(robot):
    # print(await robot.dock())
    # print(await robot.undock())
    # print('get_docking_values:', await robot.get_docking_values())
    # await robot.reset_navigation()
    # await print_pos(robot)
    # await robot.set_wheel_speeds(5, 5)
    await robot.navigate_to(0, 50)
    print("okay")
        # sensors = (await robot.get_packed_ir_proximity()).sensors
        # if front_obstacle(sensors):
        #     await backoff(robot)

    # print((await robot.get_packed_ir_proximity()).sensors)



robot.play()