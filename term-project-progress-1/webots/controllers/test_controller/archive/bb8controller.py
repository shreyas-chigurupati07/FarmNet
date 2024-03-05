from controller import Robot, Motor, GPS, LED, InertialUnit, Gyro, Compass
from simple_pid import PID
import numpy as np
import heapq
import cv2
import math


# Constants
TIME_STEP = 64
MAX_SPEED = 6.28  # Maximum speed for motors

# Define the coordinates of obstacles and cows
# These coordinates should be replaced with the actual positions in your simulation
obstacles = [(-2.19, -2.98), (0.95, -3.33), (4.05, 4.08312),
             (3.83, -3.07)]  # Placeholder for obstacle positions
cows = [(-2.4, 3.1), (-0.7, -0.6), (-0.4, -1.9),
        (2.3, 3.6)]  # Actual cow positions

# Initialize the robot
robot = Robot()

# Global constants
TURN_SPEED = 1.0  # Speed of the robot when turning to avoid an obstacle
# Threshold distance to consider an obstacle too close (in meters)
OBSTACLE_THRESHOLD = 0.2

# Global variables for motors and sensors
front_sensor = None
left_motor = None
right_motor = None

# Initialize GPS
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# Initialize Compass
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)


def initialize_robot():
    global front_sensor, left_motor, right_motor, robot
    # Initialize the front distance sensor
    front_sensor = robot.getDistanceSensor('front distance sensor name')
    front_sensor.enable(TIME_STEP)

    # Initialize wheel motors
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    # Set to infinity for velocity control
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

# Function to get the robot's current position and orientation


def get_robot_position_and_orientation():
    # Get the values from the sensors
    gps_values = gps.getValues()  # Returns [x, y, z] coordinates
    compass_values = compass.getValues()  # Returns a vector pointing north

    # Calculate the orientation of the robot
    orientation = math.atan2(compass_values[0], compass_values[1])

    # Return the 2D position (x, z in Webots as it uses a Y-up coordinate system) and orientation
    return gps_values[0], gps_values[2], orientation

# Function to get the robot's current position


def get_robot_position():
    # Get the values from the GPS sensor
    gps_values = gps.getValues()  # Returns [x, y, z] coordinates

    # Return the 2D position (x, z in Webots as it uses a Y-up coordinate system)
    return gps_values[0], gps_values[2]

# Add your navigate_to, is_near_target, and avoid_obstacle functions here.


def navigate_to(target_x, target_y):
    global left_motor, right_motor

    # Constants
    MAX_SPEED = 6.28  # Example max speed for motors

    # Robot's current position and orientation
    current_x, current_y, current_orientation = get_robot_position_and_orientation()

    # Calculate the angle to the target
    angle_to_target = math.atan2(target_y - current_y, target_x - current_x)

    # Calculate the necessary rotation to align with the target
    angle_difference = angle_to_target - current_orientation

    # Normalize the angle difference
    angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

    # Determine motor speeds based on angle difference
    if abs(angle_difference) > 0.1:  # Angle difference threshold
        left_speed = -MAX_SPEED if angle_difference > 0 else MAX_SPEED
        right_speed = MAX_SPEED if angle_difference > 0 else -MAX_SPEED
    else:
        # If facing the target, move towards it
        distance_to_target = math.hypot(
            target_x - current_x, target_y - current_y)
        # Distance threshold
        left_speed = right_speed = MAX_SPEED if distance_to_target > 0.05 else 0

    # Set the motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


def is_near_target(target_x, target_y):
    NEAR_THRESHOLD = 0.1  # Distance threshold to define 'near' (in meters)

    # Obtain the robot's current position
    current_x, current_y = get_robot_position()

    # Calculate the squared distance to the target to avoid using math.sqrt for efficiency
    squared_distance_to_target = (
        target_x - current_x) ** 2 + (target_y - current_y) ** 2
    squared_threshold = NEAR_THRESHOLD ** 2

    # Check if the robot is within the 'near' threshold of the target
    return squared_distance_to_target <= squared_threshold


def avoid_obstacle():
    global front_sensor, left_motor, right_motor

    # Read sensor value
    front_distance = front_sensor.getValue()

    # Check if the front sensor detects an obstacle
    if front_distance < OBSTACLE_THRESHOLD:
        # Obstacle detected, turn the robot
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)
    else:
        # No obstacle, proceed normally (adjust this according to your robot's normal behavior)
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
# Main control loop


def main():
    global robot
    robot = Robot()
    initialize_robot()

    while robot.step(TIME_STEP) != -1:
        # Replace with your condition or mode check
        for cow in cows:
            navigate_to(cow[0], cow[1])
            if is_near_target(cow[0], cow[1]):
                print("Reached cow at position:", cow)
                # Perform additional actions
                break  # Remove this if you want to continue to the next cow

            avoid_obstacle()


# Call the main function to start the simulation
if __name__ == "__main__":
    main()
