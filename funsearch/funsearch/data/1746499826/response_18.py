import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Ensure reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1`, incorporating hybrid heuristic.

    Hybrid heuristic:
        - Use nearest neighbor to initialize the route.
        - Apply k-opt (k=2) with probability 0.5 to improve the route.

    """
    # Initialize route using nearest neighbor
    route = nearest_neighbor(_distances)

    # Apply hybrid heuristic with probability 0.5
    if np.random.rand() < 0.5:
        route = k_opt(route, _distances, k=2)

    return route
