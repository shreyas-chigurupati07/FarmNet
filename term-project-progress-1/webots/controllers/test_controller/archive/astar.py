"""crazyflie controller with autonomous A* navigation."""

import math
import heapq
import pid_controller


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def a_star(grid, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize both open and closed list
    open_list = []
    closed_list = set()

    # Heapify the open_list and add the start node
    heapq.heappush(open_list, (start_node.f, start_node))

    # Loop until the open_list is empty
    while open_list:
        _, current_node = heapq.heappop(open_list)
        closed_list.add(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) **
                       2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node[1] and child.g > open_node[1].g:
                    continue

            # Add the child to the open list
            heapq.heappush(open_list, (child.f, child))
    return None


def world_to_grid(world_coord, WORLD_SCALE, GRID_OFFSET):
    """Converts world coordinates (x, y) to grid coordinates."""
    grid_x = int((world_coord[0] / WORLD_SCALE) + GRID_OFFSET[0])
    grid_y = int((world_coord[1] / WORLD_SCALE) + GRID_OFFSET[1])
    return (grid_x, grid_y)


def grid_to_world(grid_coord, WORLD_SCALE, GRID_OFFSET):
    """Converts grid coordinates (x, y) to world coordinates."""
    world_x = (grid_coord[0] - GRID_OFFSET[0]) * WORLD_SCALE
    world_y = (grid_coord[1] - GRID_OFFSET[1]) * WORLD_SCALE
    return (world_x, world_y)
