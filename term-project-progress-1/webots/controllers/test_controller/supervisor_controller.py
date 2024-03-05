from controller import Robot, Motor, GPS, Compass, DistanceSensor
import math

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28  # Maximum speed for motors
TURN_SPEED = 1.0  # Speed of the robot when turning to avoid an obstacle
OBSTACLE_THRESHOLD = 0.2  # Distance threshold for obstacle detection

# Coordinates of obstacles and cows
obstacles = [(-2.19, -2.98), (0.95, -3.33), (4.05, 4.08312), (3.83, -3.07)]
cows = [(-2.4, 3.1), (-0.7, -0.6), (-0.4, -1.9), (2.3, 3.6)]

# Initialize the robot
robot = Robot()

# Initialize motors for internal weight shifting
body_yaw_motor = robot.getDevice('body yaw motor')
body_pitch_motor = robot.getDevice('body pitch motor')
body_yaw_motor.setPosition(float('inf'))
body_pitch_motor.setPosition(float('inf'))

# Initialize GPS and Compass
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

# Initialize the Distance Sensor
front_sensor = robot.getDevice('front distance sensor')
front_sensor.enable(TIME_STEP)


def get_robot_position_and_orientation():
    gps_values = gps.getValues()
    compass_values = compass.getValues()
    orientation = math.atan2(compass_values[0], compass_values[1])
    return gps_values[0], gps_values[2], orientation


def navigate_to(target_x, target_y):
    current_x, current_y, _ = get_robot_position_and_orientation()
    angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
    body_yaw_motor.setVelocity(math.cos(angle_to_target) * MAX_SPEED)
    body_pitch_motor.setVelocity(math.sin(angle_to_target) * MAX_SPEED)


def is_near_target(target_x, target_y):
    current_x, current_y = get_robot_position_and_orientation()[:2]
    return (target_x - current_x)**2 + (target_y - current_y)**2 <= 0.1**2


def avoid_obstacle():
    front_distance = front_sensor.getValue()
    if front_distance < OBSTACLE_THRESHOLD:
        body_yaw_motor.setVelocity(-TURN_SPEED)
        body_pitch_motor.setVelocity(TURN_SPEED)
    else:
        body_yaw_motor.setVelocity(0)
        body_pitch_motor.setVelocity(0)


def main():
    while robot.step(TIME_STEP) != -1:
        for cow in cows:
            navigate_to(cow[0], cow[1])
            if is_near_target(cow[0], cow[1]):
                print("Reached cow at position:", cow)
                break
            avoid_obstacle()


if __name__ == "__main__":
    main()
