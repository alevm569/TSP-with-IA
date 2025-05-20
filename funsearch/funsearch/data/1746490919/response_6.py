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

    # Implement a hybrid heuristic that combines two or more of the following:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - hybrid or novel heuristics of your own design

    # Example hybrid heuristic:
    # 1. Use nearest neighbor to generate an initial route.
    # 2. Perform local search to improve the route.

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    # Return the best route found.
    return tuple(range(len(_distances)))
