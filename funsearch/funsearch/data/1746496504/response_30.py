import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid heuristic combining nearest neighbor and 2-opt
    start_city = 0
    route = np.zeros(len(_distances), dtype=int)
    route[0] = start_city

    # Nearest neighbor heuristic
    current_city = start_city
    for i in range(1, len(_distances)):
        nearest_city = np.argmin(_distances[current_city])
        route[i] = nearest_city
        current_city = nearest_city

    # 2-opt heuristic
    for i in range(len(_distances)):
        for j in range(i + 1, len(_distances)):
            if calculate_route_distance(route, _distances) > calculate_route_distance(np.roll(route, j - i), _distances):
                route = np.roll(route, j - i)

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
