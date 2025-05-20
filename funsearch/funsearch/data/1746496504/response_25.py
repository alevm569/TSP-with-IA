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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor
    - Cheapest insertion

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize variables
    n = len(distances)
    visited = np.zeros(n, dtype=bool)
    route = np.zeros(n + 1, dtype=int)

    # Start from the first city
    current_city = 0
    visited[current_city] = True

    # Build the route
    for i in range(n):
        # Find the nearest unvisited city
        nearest_city = -1
        min_distance = math.inf
        for j in range(n):
            if not visited[j] and distances[current_city][j] < min_distance:
                nearest_city = j
                min_distance = distances[current_city][j]

        # Add the nearest city to the route
        route[i] = nearest_city
        visited[nearest_city] = True
        current_city = nearest_city

    # Return to the starting city
    route[n] = 0

    return tuple(route)
