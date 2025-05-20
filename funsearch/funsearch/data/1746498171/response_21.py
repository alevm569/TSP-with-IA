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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize random seed for reproducibility
    np.random.seed(42)

    # Define hybrid heuristic that combines multiple strategies
    def hybrid_heuristic(distances):
        # Use nearest neighbor to initialize a partial route
        partial_route = funsearch.nearest_neighbor(distances)

        # Use cheapest insertion to fill in the remaining cities
        partial_route = funsearch.cheapest_insertion(distances, partial_route)

        # Use local search to improve the route
        partial_route = funsearch.local_search(distances, partial_route)

        return partial_route

    # Use hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

    return best_route
