import math
import numpy as np
import heapq

MOE = 0.05  # margin of error

class Node:
    """A node class for A* Pathfinding in 3D space"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0  # g (cost) = cost of path from start node to end node
        # h (Heuristic) = estimate of cost from to current node to goal
        self.h = 0
        self.f = 0  # f (total cost) = g + h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(tuple(self.position))  # Hash the position list for set operations

def child_in_open_list(child, open_list):
    for _, open_node in open_list:
        if child == open_node and child.g > open_node.g:
            return True
    return False

def a_star(grid, start, end):
    """Returns a list of lists as a path from the given start to the given end in a 3D grid"""

    # Initialize both open and closed list
    open_list = []
    closed_list = set()

    # Create start and end node
    start_node = Node(None, start)  # Start is now a list (x, y, z)
    end_node = Node(None, end)      # End is now a list (x, y, z)

    print("Starting Position -", start)
    print("Goal Position -", end)
    print("Grid dimensions -", np.shape(grid))

    # Heapify the open_list and add the start node
    heapq.heappush(open_list, (start_node.f, start_node))

    # Loop until the open_list is empty
    while open_list:
        _, current_node = heapq.heappop(open_list)
        closed_list.add(tuple(current_node.position))  # Convert to tuple before adding

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
                print("Current -", current)
            return path

        children = []
        # Checks for the neighboring nodes
        for new_position in [(0, -0.1, 0), (0, 0.1, 0), (-0.1, 0, 0), (0.1, 0, 0), (0, 0, -0.05), (0, 0, 0.05)]:
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1],
                             current_node.position[2] + new_position[2])

            # snap node coordinates to grid
            node_closest = 999999
            node_closestPos = []
            for pos in grid:
                node_distance = math.dist(node_position, pos)

                if node_distance < node_closest:
                    node_closest = node_distance
                    node_closestPos = pos

            node_position = node_closestPos
            print("New position -", node_position)

            if not is_position_valid(grid, node_position):
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        print("Children -", tuple(children[0].position))
        # return [[0, 0, 0]]

        for child in children:

            if tuple(child.position) in closed_list:
                continue

            child.g = current_node.g + 1
            # Calculate h considering x, y, z
            child.h = sum(
                [(child.position[i] - end_node.position[i]) ** 2 for i in range(3)])
            child.f = child.g + child.h

            if not child_in_open_list(child, open_list):
                heapq.heappush(open_list, (child.f, child))

        print("Calculating A* -", current_node.position)

    return None


def position2ID(position):
    return str(position[0])+","+str(position[1])+","+str(position[2])

def a_star2(grid, start, end):
    possibilities = {}
    possibilities[position2ID(start)] = {"position":start, "FromStart":0, "FromFinish":math.dist(start, end), "difficulty":math.dist(start, end)}
    analyzed = {}
    found = False
    while found == False:
        current = 1
        ID = 1
        for i, d in possibilities.items():
            current = d
            ID = i
            break
        for i, option in possibilities.items():
            if option["difficulty"] <= current["difficulty"] and option["FromFinish"] < current["FromFinish"]:
                current = option
                ID = i
        del possibilities[ID]
        analyzed[position2ID(current["position"])] = current
        if current["position"] == end:
            found = True
            break

        neighbors = []
        for new_position in [(0, -0.1, 0), (0, 0.1, 0), (-0.1, 0, 0), (0.1, 0, 0), (0, 0, -0.05), (0, 0, 0.05)]:
            node_position = (current["position"][0] + new_position[0],
                             current["position"][1] + new_position[1],
                             current["position"][2] + new_position[2])
            # snap node coordinates to grid
            node_closest = 999999
            node_closestPos = []
            for pos in grid:
                node_distance = math.dist(node_position, pos)

                if node_distance < node_closest:
                    node_closest = node_distance
                    node_closestPos = pos

            node_position = node_closestPos
            if is_position_valid(grid, node_position):
                neighbors.append(node_position)

        for neighbor in neighbors:
            FromStart = current["FromStart"] + math.dist(current["position"], neighbor)
            if (not position2ID(neighbor) in analyzed) and ((not position2ID(neighbor) in possibilities) or possibilities[position2ID(neighbor)]["FromStart"] > FromStart):
                difficulty = FromStart + math.dist(neighbor, end)
                possibilities[position2ID(neighbor)] = {"position":neighbor, "FromStart":FromStart, "FromFinish":math.dist(neighbor, end), "difficulty":difficulty, "parent":current["position"]}

    if found == True:
        backtrack = False
        path = []
        WhereDidYouComeFrom = end
        while backtrack == False:
            WhereDidYouComeFrom = analyzed[position2ID(WhereDidYouComeFrom)]["parent"]
            path.append(WhereDidYouComeFrom)
            if WhereDidYouComeFrom == start:
                backtrack = True
        #print(path)
        return path



def error(des, now_):
    return abs(des - now_)

def is_position_valid(grid, position):
    """Check if a given position is valid in the grid"""
    drone_x, drone_y, drone_z = position

    #print(np.shape(grid))
    #if x < 0 or y < 0 or z < 0 or x >= len(grid) or y >= len(grid[0]) or z >= len(grid[0][0]):
    # if x < 0 or x >= len(grid):
    #     return False
    # if y < 0 or y >= len(grid[0]):
    #     return False
    # if z < 0 or z >= len(grid[0][0]):  # Additional check for z-dimension
    # if z < 0:  # Additional check for z-dimension
    #     return False
    # if [x, y, z] not in grid:
    #     print(str([x, y, z]) + " not in grid")
    #     return False
    for pos in grid:
        grid_x = pos[0]
        grid_y = pos[1]
        grid_z = pos[2]

        if (error(drone_x, grid_x) < MOE) and (error(drone_y, grid_y) < MOE) and (error(drone_z, grid_z) < MOE):
            # is valid
            return True  # Assuming 0 indicates walkable/valid

    return False


def child_in_open_list(child, open_list):
    """Check if a child node is in the open list"""
    for open_node in open_list:
        if child == open_node[1] and child.g > open_node[1].g:
            return True
    return False

def scale_and_interpolate(path, scaling_factor, num_intermediate_points=5):
    """
    Scales down the values in the path based on the scaling factor and generates intermediate points.
    Rounds values to 2 decimal places.

    :param path: List of lists representing the path.
    :param scaling_factor: Factor by which to scale down the path values.
    :param num_intermediate_points: Number of intermediate points to generate between each pair of path points.
    :return: A new path with scaled, interpolated, and rounded points.
    """
    # Normalize and round the path using the scaling factor
    scaled_path = [(round(x * scaling_factor, 2), round(y * scaling_factor,
                    2), round(z * scaling_factor, 2)) for x, y, z in path]

    # Generate and round intermediate points
    new_path = []
    for i in range(len(scaled_path) - 1):
        start = scaled_path[i]
        end = scaled_path[i + 1]
        new_path.append(start)
        for j in range(1, num_intermediate_points + 1):
            fraction = j / (num_intermediate_points + 1)
            intermediate_point = list(
                round(start[k] + fraction * (end[k] - start[k]), 2) for k in range(3))
            new_path.append(intermediate_point)
    new_path.append(scaled_path[-1])

    return new_path

def world_to_grid(world_coord, WORLD_SCALE, GRID_OFFSET):
    """Converts world coordinates (x, y, z) to grid coordinates."""
    grid_x = int((world_coord[0] / WORLD_SCALE) + GRID_OFFSET[0])
    grid_y = int((world_coord[1] / WORLD_SCALE) + GRID_OFFSET[1])
    grid_z = int((world_coord[2] / WORLD_SCALE) + GRID_OFFSET[2])  # Added for z-coordinate
    return [grid_x, grid_y, grid_z]


def grid_to_world(grid_coord, WORLD_SCALE, GRID_OFFSET):
    """Converts grid coordinates (x, y, z) to world coordinates."""
    world_x = (grid_coord[0] - GRID_OFFSET[0]) * WORLD_SCALE
    world_y = (grid_coord[1] - GRID_OFFSET[1]) * WORLD_SCALE
    world_z = (grid_coord[2] - GRID_OFFSET[2]) * WORLD_SCALE  # Added for z-coordinate
    return [world_x, world_y, world_z]
