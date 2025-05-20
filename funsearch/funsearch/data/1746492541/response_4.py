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

    # Use k-opt heuristic with a seed for reproducibility
    np.random.seed(42)
    best_route = funsearch.kopt(_distances)

    # Ensure route includes all cities and returns to the starting point
    if not is_valid_route(best_route, _distances):
        raise ValueError("Invalid route generated.")

    return best_route


def is_valid_route(route: tuple[int, ...], distances: np.ndarray) -> bool:
    """Checks if a route is valid, i.e., includes all cities and returns to the starting point."""
    if len(route) != len(distances):
        return False
    for i in range(len(route)):
        if route[i] not in range(len(distances)):
            return False
    return route[0] == route[-1]
