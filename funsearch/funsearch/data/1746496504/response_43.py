import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set random seed for reproducibility

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

    # Use a hybrid approach combining nearest neighbor and k-opt
    route = funsearch.nearest_neighbor(_distances)
    route = funsearch.k_opt(route, _distances, k=2)

    # Ensure route includes all cities exactly once and returns to the starting point
    if not funsearch.is_valid_route(route, _distances):
        raise ValueError("Invalid route detected.")

    return route
