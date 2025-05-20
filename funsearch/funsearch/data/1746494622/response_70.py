import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

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

    # Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics
    route = funsearch.hybrid_heuristic(
        _distances,
        funsearch.nearest_neighbor,
        funsearch.cheapest_insertion,
    )

    # Perform local search to improve the solution
    route = funsearch.local_search(route, _distances)

    return route
