import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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

    # Apply 2-opt heuristic to generate an initial route
    route = funsearch.two_opt(_distances)

    # Perform local search to refine the route
    route = funsearch.local_search(_distances, route)

    # Ensure route includes all cities and returns to the starting point
    if not funsearch.is_valid_route(route, _distances):
        raise ValueError("Invalid route.")

    return route
