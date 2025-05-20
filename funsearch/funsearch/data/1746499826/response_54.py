import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Apply nearest neighbor heuristic to get an initial route
    start_city = np.random.randint(len(matrix_distances))
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(matrix_distances):
        current_city = route[-1]
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Use 2-opt heuristic to improve the route
    for i in range(100):
        for j in range(len(route)):
            for k in range(j + 2, len(route)):
                if calculate_route_distance(route, matrix_distances) > calculate_route_distance(route[j:k][::-1] + route[:j] + route[k:], matrix_distances):
                    route = route[j:k][::-1] + route[:j] + route[k:]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """

    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return distance
