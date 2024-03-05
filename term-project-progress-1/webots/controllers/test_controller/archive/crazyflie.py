"""crazyflie controller."""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, DistanceSensor, Camera
from astar import a_star, world_to_grid, grid_to_world
import math
# External controller
import pid_controller

# Constants for the A* algorithm
FLYING_ALTITUDE = 1.0
WORLD_SCALE = 0.1
GRID_OFFSET = (50, 50)
GRID_SIZE = (int(10 / 0.1), int(10 / 0.1))  # (100, 100)


# Now you can define OBSTACLES and GOALS using the world_to_grid function
# Replace these with the actual world coordinates of your obstacles and goals
OBSTACLES = [
    world_to_grid((-2.19, -2.98), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((0.95, -3.33), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((4.05, 4.08312), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((3.83, -3.07), WORLD_SCALE, GRID_OFFSET)
]

GOALS = [
    world_to_grid((-2.4, 3.1), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((-0.7, -0.6), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((-0.4, -1.9), WORLD_SCALE, GRID_OFFSET),
    world_to_grid((2.3, 3.6), WORLD_SCALE, GRID_OFFSET)
]


# def main():
#     robot = Robot()
#     timestep = int(robot.getBasicTimeStep())

#     # Initialize motors
#     m1_motor = robot.getDevice("m1_motor")
#     m1_motor.setPosition(float('inf'))
#     m1_motor.setVelocity(-1.0)
#     m2_motor = robot.getDevice("m2_motor")
#     m2_motor.setPosition(float('inf'))
#     m2_motor.setVelocity(1.0)
#     m3_motor = robot.getDevice("m3_motor")
#     m3_motor.setPosition(float('inf'))
#     m3_motor.setVelocity(-1.0)
#     m4_motor = robot.getDevice("m4_motor")
#     m4_motor.setPosition(float('inf'))
#     m4_motor.setVelocity(1.0)

#     # Initialize sensors
#     imu = robot.getDevice("inertial_unit")
#     imu.enable(timestep)
#     gps = robot.getDevice("gps")
#     gps.enable(timestep)
#     keyboard = Keyboard()
#     keyboard.enable(timestep)
#     gyro = robot.getDevice("gyro")
#     gyro.enable(timestep)
#     camera = robot.getDevice("camera")
#     camera.enable(timestep)
#     range_front = robot.getDevice("range_front")
#     range_front.enable(timestep)
#     range_left = robot.getDevice("range_left")
#     range_left.enable(timestep)
#     range_back = robot.getDevice("range_back")
#     range_back.enable(timestep)
#     range_right = robot.getDevice("range_right")
#     range_right.enable(timestep)

#     # Wait for 2 seconds
#     start_time = robot.getTime()
#     while robot.step(timestep) != -1:
#         if robot.getTime() - start_time > 2.0:
#             break

#     # Initialize variables
#     actual_state = pid_controller.ActualState()
#     desired_state = pid_controller.DesiredState()
#     gains_pid = pid_controller.GainsPID()
#     motor_power = pid_controller.MotorPower()
#     control_commands = pid_controller.ControlCommands()
#     past_x_global = 0
#     past_y_global = 0
#     past_time = robot.getTime()

#     # Initialize PID gains.
#     gains_pid.kp_att_y = 1
#     gains_pid.kd_att_y = 0.5
#     gains_pid.kp_att_rp = 0.5
#     gains_pid.kd_att_rp = 0.1
#     gains_pid.kp_vel_xy = 2
#     gains_pid.kd_vel_xy = 0.5
#     gains_pid.kp_z = 10
#     gains_pid.ki_z = 5
#     gains_pid.kd_z = 5
#     pid_controller.init_pid_attitude_fixed_height_controller()

#     height_desired = FLYING_ALTITUDE

#     # Initialize struct for motor power
#     print("\n====== Controls =======\n")
#     print(" The Crazyflie can be controlled from your keyboard!\n")
#     print(" All controllable movement is in body coordinates\n")
#     print("- Use the up, back, right and left button to move in the horizontal plane\n")
#     print("- Use Q and E to rotate around yaw\n ")
#     print("- Use W and S to go up and down\n")

#     while robot.step(timestep) != -1:
#         dt = robot.getTime() - past_time

#         # Get measurements
#         actual_state.roll, actual_state.pitch, actualYaw = imu.getRollPitchYaw()
#         actual_state.yaw_rate = gyro.getValues()[2]
#         actual_state.altitude = gps.getValues()[2]
#         x_global = gps.getValues()[0]
#         vx_global = (x_global - past_x_global) / dt
#         y_global = gps.getValues()[1]
#         vy_global = (y_global - past_y_global) / dt

#         # Get body fixed velocities
#         cosyaw = math.cos(actualYaw)
#         sinyaw = math.sin(actualYaw)
#         actual_state.vx = vx_global * cosyaw + vy_global * sinyaw
#         actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw

#         # Initialize values
#         desired_state.roll = 0
#         desired_state.pitch = 0
#         desired_state.vx = 0
#         desired_state.vy = 0
#         desired_state.yaw_rate = 0
#         desired_state.altitude = 1.0

#         forward_desired = 0
#         sideways_desired = 0
#         yaw_desired = 0
#         height_diff_desired = 0

#         # Control altitude
#         key = keyboard.getKey()
#         while key > 0:
#             if key == Keyboard.UP:
#                 forward_desired = +0.5
#             elif key == Keyboard.DOWN:
#                 forward_desired = -0.5
#             elif key == Keyboard.RIGHT:
#                 sideways_desired = -0.5
#             elif key == Keyboard.LEFT:
#                 sideways_desired = +0.5
#             elif key == ord('Q'):
#                 yaw_desired = 1.0
#             elif key == ord('E'):
#                 yaw_desired = -1.0
#             elif key == ord('W'):
#                 height_diff_desired = 0.1
#             elif key == ord('S'):
#                 height_diff_desired = -0.1
#             key = keyboard.getKey()

#         height_desired += height_diff_desired * dt

#         desired_state.yaw_rate = yaw_desired

#         # PID velocity controller with fixed height
#         desired_state.vy = sideways_desired
#         desired_state.vx = forward_desired
#         desired_state.altitude = height_desired
#         pid_controller.pid_velocity_fixed_height_controller(
#             actual_state, desired_state, gains_pid, dt, motor_power)

#         # Setting motorspeed
#         m1_motor.setVelocity(-motor_power.m1)
#         m2_motor.setVelocity(motor_power.m2)
#         m3_motor.setVelocity(-motor_power.m3)
#         m4_motor.setVelocity(motor_power.m4)

#         # Save past time for next time step
#         past_time = robot.getTime()
#         past_x_global = x_global
#         past_y_global = y_global

#     robot.cleanup()


def navigate_to_point(robot, point, timestep):
    """Navigate the quadrotor to a given point."""
    # PID controller setup
    gains_pid = pid_controller.gains_pid_t()
    gains_pid.kp_att_y = 1
    gains_pid.kd_att_y = 0.5
    gains_pid.kp_att_rp = 0.5
    gains_pid.kd_att_rp = 0.1
    gains_pid.kp_vel_xy = 2
    gains_pid.kd_vel_xy = 0.5
    gains_pid.kp_z = 10
    gains_pid.ki_z = 5
    gains_pid.kd_z = 5

    # Initialize motor power struct
    motor_power = pid_controller.motor_power_t()

    # Initialize state variables
    actual_state = pid_controller.actual_state_t()
    desired_state = pid_controller.desired_state_t()
    desired_state.altitude = FLYING_ALTITUDE

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    # Motor devices
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))

    # Main control loop for reaching a single point
    while robot.step(timestep) != -1:
        # Current state update
        actual_state.roll, actual_state.pitch, actual_state.yaw = imu.getRollPitchYaw()
        actual_state.yaw_rate = gyro.getValues()[2]
        actual_state.altitude = gps.getValues()[2]
        actual_state.vx, actual_state.vy = 0, 0  # Assuming stationary for simplicity

        # Desired state update for reaching the point
        desired_state.x = point[0]
        desired_state.y = point[1]

        # Call the PID controller to compute new motor powers
        pid_controller.pid_velocity_fixed_height_controller(
            actual_state, desired_state, gains_pid, timestep, motor_power)

        # Set motor speeds based on controller output
        m1_motor.setVelocity(-motor_power.m1)
        m2_motor.setVelocity(motor_power.m2)
        m3_motor.setVelocity(-motor_power.m3)
        m4_motor.setVelocity(motor_power.m4)

        # Break the loop once the point is reached, or add your own condition
        if (abs(gps.getValues()[0] - point[0]) < 0.1 and
                abs(gps.getValues()[1] - point[1]) < 0.1):
            break


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m2_motor = robot.getDevice("m2_motor")
    m3_motor = robot.getDevice("m3_motor")
    m4_motor = robot.getDevice("m4_motor")
    motors = [m1_motor, m2_motor, m3_motor, m4_motor]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    # (initialize other sensors if needed)

    # Initialize PID controller (if necessary)
    pid_controller.init_pid_attitude_fixed_height_controller()

    # Setup the grid with obstacles
    grid = [[0 for _ in range(GRID_SIZE[1])] for _ in range(GRID_SIZE[0])]
    for obstacle in OBSTACLES:
        grid_x, grid_y = world_to_grid(obstacle, WORLD_SCALE, GRID_OFFSET)

        # Check if grid coordinates are within bounds
        if 0 <= grid_x < GRID_SIZE[0] and 0 <= grid_y < GRID_SIZE[1]:
            grid[grid_x][grid_y] = 1  # Mark grid cell as obstacle
        else:
            print(
                f"Obstacle at {obstacle} is out of grid bounds and will be ignored.")

    # Main control loop for autonomous navigation
    for goal_world in GOALS:  # GOALS are in world coordinates
        goal_grid = world_to_grid(goal_world, WORLD_SCALE, GRID_OFFSET)
        while robot.step(timestep) != -1:
            current_world_pos = (gps.getValues()[0], gps.getValues()[1])
            current_grid_pos = world_to_grid(
                current_world_pos, WORLD_SCALE, GRID_OFFSET)
            path = a_star(grid, current_grid_pos, goal_grid)
            if not path:
                print("No path found to the goal:", goal_world)
                break  # Skip to the next goal if no path is found

            # Follow the path
            for waypoint_grid in path[1:]:  # Skip the current position
                waypoint_world = grid_to_world(
                    waypoint_grid, WORLD_SCALE, GRID_OFFSET)
                if not navigate_to_point(robot, waypoint_world, timestep, motors):
                    print("Navigation to waypoint failed:", waypoint_world)
                    break  # Stop trying to navigate if navigation fails

            # Goal is reached; move to the next goal
            break

    robot.cleanup()


if __name__ == "__main__":
    main()
