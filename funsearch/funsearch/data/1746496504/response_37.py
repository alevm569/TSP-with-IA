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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a combination of heuristics
    best_route = funsearch.greedy_heuristic(_distances, funsearch.nearest_neighbor)
    best_route = funsearch.local_search(best_route, _distances)

    # Ensure that the route includes all cities and returns to the starting point
    assert len(np.unique(best_route)) == len(_distances)
    assert best_route[0] == best_route[-1]

    return best_route
