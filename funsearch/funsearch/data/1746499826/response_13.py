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

    # Use hybrid heuristic that combines the nearest neighbor and 2-opt heuristics
    route = funsearch.nearest_neighbor(_distances)
    funsearch.two_opt(route, _distances)

    # Ensure the route includes all cities and returns to the starting point
    if not funsearch.is_valid(route, _distances):
        raise ValueError("Invalid route.")

    return route
