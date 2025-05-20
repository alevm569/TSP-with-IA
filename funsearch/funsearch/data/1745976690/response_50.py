import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid Heuristic:
        - Uses the nearest neighbor heuristic to initialize a partial route.
        - Applies the 2-opt heuristic iteratively to improve the route.

    Args:
        matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        A tuple of city indices representing the best route.
    """

    # Initialize a partial route using the nearest neighbor heuristic.
    start_city = 0
    current_city = start_city
    partial_route = [current_city]

    while len(partial_route) < len(matrix_distances):
        nearest_city = np.argmin(matrix_distances[current_city])
        if nearest_city not in partial_route:
            partial_route.append(nearest_city)
            current_city = nearest_city

    # Apply the 2-opt heuristic iteratively to improve the route.
    for _ in range(100):
        for i in range(len(partial_route)):
            for j in range(i + 1, len(partial_route)):
                distance_difference = matrix_distances[partial_route[i]][partial_route[(j + 1) % len(partial_route)]] + \
                                    matrix_distances[partial_route[j]][partial_route[(i + 1) % len(partial_route)]] - \
                                    matrix_distances[partial_route[i]][partial_route[j]]

                if distance_difference < 0:
                    partial_route = partial_route[:i] + partial_route[j:i:-1] + partial_route[j + 1:]

    # Return the best route as a tuple of city indices.
    return tuple(partial_route)
