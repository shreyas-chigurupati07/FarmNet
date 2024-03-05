from controller import Supervisor
import csv
import math
import random
import numpy as np
#print(ShaftTranslation.getSFVec3f())
#print(ShaftRotation.getSFRotation())
#ShaftTranslation.setSFVec3f(StartPosition)
#ShaftRotation.setSFRotation(StartRotation)
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
#DroneNode = supervisor.getFromDef("MainShaft")
#DronePosition = DroneNode.getField('translation')
#supervisor.getRoot().getField('children').importMFNodeFromString(6, GeneratePointNodeString("test", DronePosition, 0.1, [1,0,0]))

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

def RandomPoint():
    return [random.randrange(-100, 100)*0.01, random.randrange(-100, 100)*0.01, random.randrange(10, 50)*0.01]

def DistanceBetween(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def ScannerCollides():
    ContactPoints = ScannerNode.getContactPoints()
    if len(ContactPoints) > 0: # if there is at least 1 contact point, the mainshaft collides with something
        return True
    else:
        return False

ScannerNodeString = '''DEF scanner Solid {
    children [
        Shape {
            appearance DEF ScannerColor PBRAppearance {
            metalness 0
            }
            geometry Sphere {
            radius 0.1
            }
            castShadows FALSE
        }
    ]
    boundingObject Sphere {
        radius 0.1
    }
    physics Physics {
    }
}'''

# supervisor setup
supervisor = Supervisor()
timestep = 32

# Get node info of quadcopter
# DroneNode = supervisor.getFromDef("drone")
# DronePosition = DroneNode.getField('translation')
# DronePosition.setSFVec3f([0,0,-0.5])

# Make dot node group
supervisor.getRoot().getField('children').importMFNodeFromString(-1, "DEF PathDots Group {}")
DotGroupChildren = supervisor.getFromDef("PathDots").getField('children')

# Make start and end dots
#supervisor.getRoot().getField('children').importMFNodeFromString(-1, GeneratePointNodeString("Start", RandomPoint(), 0.1, [1, 0, 0]))
#supervisor.getRoot().getField('children').importMFNodeFromString(-1, GeneratePointNodeString("Goal", RandomPoint(), 0.1, [0, 1, 0]))

# Make scanner
supervisor.getRoot().getField('children').importMFNodeFromString(-1, ScannerNodeString)
ScannerNode = supervisor.getFromDef("scanner")
ScannerPositionField = ScannerNode.getField('translation')
ScannerColorField = supervisor.getFromDef("ScannerColor").getField('baseColor')
ScannerNode.enableContactPointsTracking(timestep)

ArenaNode = supervisor.getFromDef("farmland")
ArenaSizeField = ArenaNode.getField('floorSize')
ArenaDimensions = ArenaSizeField.getSFVec2f()


# possible points visualization
AllPositions = []
for x in range(-math.floor(ArenaDimensions[0])*5, (math.floor(ArenaDimensions[0])*5) + 1, 1):
    for y in range(-math.floor(ArenaDimensions[1])*5, (math.floor(ArenaDimensions[1])*5) + 1, 1):
        for z in range(5, 60, 5):
            AllPositions.append([x/10, y/10, z/100])
            #DotGroupChildren.importMFNodeFromString(-1, GeneratePointNodeString("GridDot", [x/10, y/10, z/10], 0.05, [0, 0, 0.5]))
ScannerPositionField.setSFVec3f(AllPositions[0])

state = "NewNode"
i = 1
GridFormed = False
GridPositions = []
VisualizeField = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # if GridFormed == True and i%10 == 0:
    if GridFormed == True:
        # DotGroupChildren.importMFNodeFromString(-1, GeneratePointNodeString("Dot", DronePosition.getSFVec3f(), 0.1, rainbow(i, 1000)))
        break

    if GridFormed == False:
        # DronePosition.setSFVec3f([0,0,-0.1])
        # DroneNode.setVelocity([0,0,0,0,0,0])
        if ScannerCollides() == True:
            if VisualizeField: DotGroupChildren.importMFNodeFromString(-1, GeneratePointNodeString("GridDot", AllPositions[0], 0.1, [1, 0, 0]))
        else:
            if VisualizeField: DotGroupChildren.importMFNodeFromString(-1, GeneratePointNodeString("GridDot", AllPositions[0], 0.05, [0, 1, 0]))
            GridPositions.append(AllPositions[0])
        AllPositions.pop(0)
        if len(AllPositions) > 0:
            ScannerPositionField.setSFVec3f(AllPositions[0])
        else:
            GridFormed = True
            ScannerNode.remove()
            # DronePosition.setSFVec3f([0,0,0.1])
            # DroneNode.setVelocity([0,0,0,0,0,0])
            # supervisor.simulationSetMode("SIMULATION_MODE_PAUSE")
    i = i+1

if VisualizeField == False:
    OUTPUT_FILE = "freespaces.data"
    freespaceFile = open(OUTPUT_FILE, "w+")
    csvWriter = csv.writer(freespaceFile)
    csvWriter.writerows(GridPositions)
    freespaceFile.close()
    print("Grid written to -", OUTPUT_FILE)