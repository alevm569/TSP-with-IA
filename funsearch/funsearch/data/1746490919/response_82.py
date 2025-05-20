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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Use a hybrid heuristic combining local search with a 2-opt move operator
    best_route = funsearch.local_search(funsearch.random_permutation(_distances), _distances, funsearch.two_opt)

    # Ensure the route is valid
    if not funsearch.is_valid_route(best_route, _distances):
        raise ValueError("Invalid route.")

    return best_route
