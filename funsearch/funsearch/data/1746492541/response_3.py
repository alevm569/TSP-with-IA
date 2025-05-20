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

    # Use a hybrid approach combining the nearest neighbor and k-opt heuristics
    best_route = funsearch.hybrid_search(
        distance_matrix=_distances,
        initial_permutation=funsearch.nearest_neighbor(_distances),
        optimization_method=funsearch.k_opt,
        max_iterations=1000,
        verbose=False,
    )

    return best_route
