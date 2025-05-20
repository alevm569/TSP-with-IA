import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Implement a hybrid heuristic that combines two or more of the following strategies:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - ACO (ant colony optimization)
    # - genetic algorithms
    # - k-opt
    # - tabu search

    # Example hybrid heuristic:
    route = funsearch.nearest_neighbor(_distances)
    route = funsearch.cheapest_insertion(route, _distances)
    route = funsearch.local_search(route, _distances)

    return route
