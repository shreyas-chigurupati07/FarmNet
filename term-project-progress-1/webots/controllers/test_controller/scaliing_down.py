def scale_and_interpolate(path, scaling_factor, num_intermediate_points=5):
    """
    Scales down the values in the path based on the scaling factor and generates intermediate points.
    Rounds values to 2 decimal places.

    :param path: List of tuples representing the path.
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
            intermediate_point = tuple(
                round(start[k] + fraction * (end[k] - start[k]), 2) for k in range(3))
            new_path.append(intermediate_point)
    new_path.append(scaled_path[-1])

    return new_path


# Example usage
# Sample path from A* algorithm
path = [(0, 0, 0), (3, 4, 5), (6, 8, 10), (9, 12, 15)]
scaling_factor = 1 / 15  # Choose a scaling factor based on your requirements
scaled_path = scale_and_interpolate(path, scaling_factor)
print(scaled_path)
