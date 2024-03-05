import astar_updated as astar
from controller import Robot, Motor, GPS, LED, InertialUnit, Gyro, Compass, Supervisor
import csv
import math
import numpy as np
from simple_pid import PID
import time
import os

OPERATING_ALTITUDE = 0.5
GRID_OFFSET = [0, 0, 0]

MOE = 0.05     # margin of error
# astar.MOE = MOE

HOVER_TIME = 500

class Quadcopter:
    def __init__(self):
        self.robot = Supervisor()
        # Setup MOTOR for Quadcopter robot
        self.motor_names = ['m1_motor','m2_motor','m3_motor','m4_motor']
        self.motors = []
        for motor_names in self.motor_names:
            motor = self.robot.getDevice(motor_names)
            motor.setPosition(float('inf'))
            self.motors.append(motor)
        # Setup GPS for Quadcopter robot
        self.gps = GPS("gps")
        self.gps.enable(8)
        # Setup IMU for Quadcopter robot
        self.imu = InertialUnit("inertial unit")
        self.imu.enable(8)
        # Setup GYRO for Quadcopter robot
        self.gyro = Gyro("gyro")
        self.gyro.enable(8)
        # Setup function PID (parameters Kp,Ki,Kd not fully optimized yet)
        self.throttlePID = PID(19, 5.3 ,15, setpoint=0.1)
        self.pitchPID = PID(4.97, 0.013, 4.57, setpoint=0.1)
        self.rollPID = PID(4.92, 0.013, 4.57, setpoint=0.1)
        self.yawPID = PID(3.2, 0.013, 2.1, 0.7)

        # create current pose(Start point)
        self.current_pose = [0,0,0]

        self.pointer = 0
        self.target = [0,0,0]
        self.waypoint = []

        # open('data.txt', 'w')
        return

    def getMotorAll(self): # Get and enable 4 motor of Quadcopter robot
        m1_motor = self.robot.getDevice('m1_motor')
        m2_motor = self.robot.getDevice('m1_motor')
        m3_motor = self.robot.getDevice('m1_motor')
        m4_motor = self.robot.getDevice('m1_motor')
        return [m1_motor, m2_motor, m3_motor, m4_motor]

    def motorsSpeed(self,velocity): # Set/update velocity for 4 motor of Quadcopter robot
        motor = self.getMotorAll()
        for i in range(0,4):
            motor[i].setVelocity(velocity[i])
        return

    def updateTheseCurrent(self):
        # Orientation
        self.roll = self.imu.getRollPitchYaw()[0]
        self.pitch = self.imu.getRollPitchYaw()[1]
        self.yaw = self.imu.getRollPitchYaw()[2]

        # Acceleration  Velocities
        self.roll_acceleration = self.gyro.getValues()[0]
        self.pitch_acceleration = self.gyro.getValues()[1]

        # Position
        self.xGPS = self.gps.getValues()[0]
        self.yGPS = self.gps.getValues()[1]
        self.zGPS = self.gps.getValues()[2]

        return

    def clamp(self,value, value_min, value_max):
        return min(max(value, value_min), value_max)

    def error(self,des,now_):
        return abs(des-now_)

    def find_angle(self,target_position,current_pose):
        # a = 1 if target_position[0]>=0 else -1
        # b = 1 if target_position[0]>=0 else -1
        angle_left = (np.arctan2(target_position[0], target_position[1]) )
        angle_left = (angle_left+np.pi)%(np.pi)
        return [0,0]
        # return [angle_left,angle_left/abs(angle_left)]

    def createTrajectory(self, grid, droneCoord, destCoord):
        startTime = time.time()
        trajectory = [ [0.0, 0.0, 0.0] ]

        # number_of_reference_points = 10

        # theta = np.linspace(0, 2*np.pi, number_of_reference_points)

        # the radius of the circle
        # r = np.sqrt(0.05)

        # compute x1 and x2
        # x1 = r*np.cos(theta)
        # x2 = r*np.sin(theta)

        # for i in range(0,number_of_reference_points):
            # trajectory.append([x1[i],x2[i],0.1])

        # trajectory = [ [0.2, 0.0, 0.1],
        #                [0.2, 0.1, 0.1],
        #                [0.2, 0.2, 0.1],
        #                [0.25, 0.4, 0.1],
        #                [0.25, 0.6, 0.1],
        #                [0.30, 0.7, 0.12],
        #                [0.30, 0.9, 0.15],
        #                [0.5, 1.0, 0.2] ]

        # print("Arrary shape -", np.shape(grid))
        # print(grid)

        # Get current x,y,z coords from GPS
        # self.updateTheseCurrent()
        # xPos = self.gps.getValues()[0]
        # yPos = self.gps.getValues()[1]
        # zPos = self.gps.getValues()[2]
        # droneCoord = [ self.xGPS,
        #                self.yGPS,
        #                self.zGPS ]

        # snap drone coordinates to grid
        drone_closest = 999999
        drone_closestPos = []
        goal_closest = 999999
        goal_closestPos = []
        for pos in grid:
            droneDistance = math.dist(droneCoord, pos)
            goalDistance = math.dist(destCoord, pos)

            if droneDistance < drone_closest:
                drone_closest = droneDistance
                drone_closestPos = pos

            if goalDistance < goal_closest:
                goal_closest = goalDistance
                goal_closestPos = pos

        # print("Getting A* trajectory starting from", self.xGPS, self.yGPS, self.zGPS)
        print("Getting A* trajectory starting from", drone_closestPos, "to", goal_closestPos)

        trajectory = astar.a_star2(grid, drone_closestPos, goal_closestPos)
        trajectory.reverse()

        # simulate a hover
        for count in range(HOVER_TIME):
            trajectory.append(destCoord)

        print(trajectory)
        print("Elapsed time -", (time.time() - startTime))

        return trajectory

    def getTarget(self):
        self.updateTheseCurrent()
        target_yaw = self.find_angle(self.target,self.current_pose)
        # print(self.yaw,target_yaw[0])
        # if(self.error(target_yaw[0],self.yaw)<0.01):
            # print("true")
        if(self.error(self.target[0],self.xGPS)<MOE and self.error(self.target[1],self.yGPS)<MOE and self.error(self.target[2],self.zGPS)<MOE and self.error(target_yaw[0],self.yaw)<MOE):
            # print("pose: ",self.pointer)
            self.current_pose = self.waypoint[self.pointer-1]
            self.pointer += 1
            if self.pointer >= len(self.waypoint):
                self.pointer = len(self.waypoint) - 1
        self.target = self.waypoint[self.pointer]
        return

    def findSpeedMotor(self):# target is a waypoint
        # Update target for PID function
        self.getTarget()
        self.rollPID.setpoint = self.target[1]
        self.pitchPID.setpoint = self.target[0]
        self.throttlePID.setpoint = self.target[2]
        target_yaw = self.find_angle(self.target,self.current_pose)
        self.yawPID.setpoint = target_yaw[0]

        # Get these current position
        self.updateTheseCurrent()

        vertical_input = round(self.throttlePID(round(self.zGPS,3)),2)
        roll_input = 30*self.clamp(self.roll, -1, 1) + self.roll_acceleration  + self.rollPID(self.yGPS)
        pitch_input =  -15*self.clamp(self.pitch, -1, 1) - self.pitch_acceleration + self.pitchPID(self.xGPS)
        yaw_input = -self.yawPID((self.yaw))


        # Calc velocity for 4 motor of robot
        m1_input = round(54 + vertical_input + roll_input - pitch_input - yaw_input,3)
        m2_input = round(54 + vertical_input + roll_input + pitch_input + yaw_input,3)
        m3_input = round(54 + vertical_input - roll_input + pitch_input - yaw_input,3)
        m4_input = round(54 + vertical_input - roll_input - pitch_input + yaw_input,3)

        # Check min/max of velocity
        m1_input = self.clamp(m1_input,-600,600)
        m2_input = self.clamp(m2_input,-600,600)
        m3_input = self.clamp(m3_input,-600,600)
        m4_input = self.clamp(m4_input,-600,600)

        # Update Current pose
        return [-m1_input,m2_input,-m3_input,m4_input]

    def motorsSpeed(self,velocity):
        for i in range(0,4):
            self.motors[i].setVelocity(velocity[i])
        return

    def killMotors(self):
        for i in range(0, 4):
            self.motors[i].setVelocity(0)
        return

    def run(self):
        motor_speed = self.findSpeedMotor()
        self.motorsSpeed(motor_speed)
        # with open('data.txt', 'a+') as f:
            # f.write(str(self.xGPS) + ',' + str(self.yGPS) + ',' +  str(self.zGPS) + ',' +  str(self.yaw) + ',' + str(self.robot.getTime()) +'\n')
        print("Current position -", self.xGPS, self.yGPS, self.zGPS)
        print("Target position -", self.target)

        return

    def originHover(self):
        motor_speed = self.findSpeedMotor()
        self.motorsSpeed(motor_speed)
        # with open('data.txt', 'a+') as f:
            # f.write(str(self.xGPS) + ',' + str(self.yGPS) + ',' +  str(self.zGPS) + ',' +  str(self.yaw) + ',' + str(self.robot.getTime()) +'\n')
        print("Current position -", self.xGPS, self.yGPS, self.zGPS)
        print("Target position -", self.target)

        return

def GeneratePointNodeString(NodeNameNoSpaces, position, scale, color):
    t = "DEF "+ NodeNameNoSpaces + ''' Transform {
        translation ''' + str(position[0]) + " " + str(position[1]) + " " + str(position[2]) + '''
        scale ''' + str(scale) + " " + str(scale) + " " + str(scale) + '''
        children [
            Shape {
                appearance PBRAppearance {
                    baseColor ''' + str(color[0]) + " " + str(color[1]) + " " + str(color[2]) + '''
                    roughness 1
                    metalness 0
                    emissiveIntensity 0
                }
                geometry Sphere {
                    radius 0.1
                }
                castShadows FALSE
            }
        ]
    }'''
    return t

def rainbow(i, MaxTime):
    color = i*(4/MaxTime)
    r = 4*(math.acos(math.cos(math.pi*((color+5)/4)))/(math.pi))-1
    if r>1:
        r = 1
    elif r < 0:
        r = 0
    g = 4*(math.acos(math.cos(math.pi*((color+2)/4)))/(math.pi))-2
    if g>1:
        g = 1
    elif g < 0:
        g = 0
    b = 4*(math.acos(math.cos(math.pi*((color)/4)))/(math.pi))-2
    if b>1:
        b = 1
    elif b < 0:
        b = 0
    return [r, g, b]

if __name__ == "__main__":
    # read grid of freespaces from file
    GRID_FILE = "../supervisor/freespaces.data"
    fileReader = open(GRID_FILE, "r")
    csvRows = []
    csvReader = csv.reader(fileReader, delimiter=',')
    csvRows = list(csvReader)

    # convert strings to float
    gridFree = []
    for row in csvRows:
        if len(row) != 0:
            gridFree.append([float(row[0]), float(row[1]), float(row[2])])

    # GOALS = [[0.5, 1.0, 0.2],
    #          [1.0, 2.0, 0.2],
    #          [0.0, 0.0, 0.2]]
    # GOALS = [[0.5, 1.0, 0.2],
    #          [1.0, 2.0, 0.2]]

    drone = Quadcopter() # Create robot
    timestep = int(drone.robot.getBasicTimeStep())
    path = []

    # get initial position
    origin = [0.0, 0.0, 0.0]

    # supervisor = Supervisor()
    droneSuperv = drone.robot.getFromDef("drone")
    start = droneSuperv.getField('translation').getSFVec3f()
    print("Start Coords -", start)

    cow1_pos = drone.robot.getFromDef("Cow1").getField('translation').getSFVec3f()
    cow1_pos[2] = OPERATING_ALTITUDE
    cow2_pos = drone.robot.getFromDef("Cow2").getField('translation').getSFVec3f()
    cow2_pos[2] = OPERATING_ALTITUDE

    GOALS = [cow1_pos, cow2_pos, origin]
    print("Goals -", GOALS)

    # generate waypoints
    drone.waypoint = [[0.0, 0.0, 0.01], start, start, start, start]
    for goal in GOALS:
        reachedGoal = False
        drone.robot.step(timestep)
        drone.waypoint = drone.waypoint + drone.createTrajectory(gridFree, start, goal)
        print("Calculating Next Waypoint ...")
        start = goal    # start of next should be this goal
        # exit(0)
    drone.waypoint = drone.waypoint
    print("All waypoints generated!")

    # droneSuperv.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # second to last waypoint to indicate that it is returning to base
    finalJobCoord = GOALS[len(GOALS) - 2]

    # generates dots
    drone.robot.getRoot().getField('children').importMFNodeFromString(-1, "DEF PathDots Group {}")
    DotGroupChildren = drone.robot.getFromDef("PathDots").getField('children')
    dotStep = 1


    # run simulation
    returningToBase = False
    goalX, goalY, goalZ = drone.waypoint[len(drone.waypoint) - 1]
    while drone.robot.step(timestep) != -1: # Run the script
        # check if we are done
        curX, curY, curZ = droneSuperv.getField('translation').getSFVec3f()

        # determine when we are returning to base
        if (drone.error(curX, finalJobCoord[0]) <= MOE) and \
           (drone.error(curY, finalJobCoord[1]) <= MOE) and \
           (drone.error(curZ, finalJobCoord[2]) <= MOE):
            returningToBase = True

        # returning to base; kill motors at end
        if returningToBase and \
           (drone.error(curX, goalX) <= MOE) and \
           (drone.error(curY, goalY) <= MOE) and \
           (drone.error(curZ, goalZ) <= MOE):
            drone.killMotors()
            break

        drone.run()

        if (dotStep % 10 == 0):
            DotGroupChildren.importMFNodeFromString(-1, GeneratePointNodeString("Dot", [curX, curY, curZ], 0.1, rainbow(dotStep, 1000)))

        dotStep = dotStep + 1

    print("DONE!!!")
    exit(0)