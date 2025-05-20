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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    np.random.seed(0)  # Set a seed for reproducibility

    # Use a combination of nearest neighbor and local search heuristics
    best_route = funsearch.nearest_neighbor(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    # Perform local search to improve the route
    for _ in range(100):
        new_route = funsearch.local_search(best_route, _distances)
        new_distance = calculate_route_distance(new_route, _distances)

        if new_distance < best_distance:
            best_distance = new_distance
            best_route = new_route

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
