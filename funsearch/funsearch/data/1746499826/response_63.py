import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Set a seed for reproducibility
    np.random.seed(42)

    # Implement a hybrid heuristic that combines two or more of the following:
    # - Nearest neighbor
    # - Cheapest insertion
    # - Local search
    # - 2-opt
    # - ACO (ant colony optimization)
    # - Genetic algorithms
    # - K-opt
    # - Tabu search

    # Example hybrid heuristic:
    route = funsearch.nearest_neighbor(_distances)
    route = funsearch.cheapest_insertion(route, _distances)
    route = funsearch.local_search(route, _distances)

    return route
