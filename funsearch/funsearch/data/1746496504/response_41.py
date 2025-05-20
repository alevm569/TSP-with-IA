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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic combining the nearest neighbor and cheapest insertion heuristics
    initial_route = funsearch.nearest_neighbor(_distances)
    best_route = funsearch.cheapest_insertion(initial_route, _distances)

    # Perform local search with a 2-opt neighborhood operator
    best_route = funsearch.local_search(best_route, _distances, neighborhood_operator=funsearch.two_opt)

    return best_route
