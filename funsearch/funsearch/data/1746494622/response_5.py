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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Use a combination of nearest neighbor and cheapest insertion
    route = funsearch.nearest_neighbor(_distances)
    funsearch.cheapest_insertion(route, _distances)

    # Local search with 2-opt and k-opt for optimization
    funsearch.local_search(route, _distances, [funsearch.two_opt, funsearch.k_opt])

    return route
