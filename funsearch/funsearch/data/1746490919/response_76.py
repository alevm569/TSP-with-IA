import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic or combine two or more heuristics from the list below:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - hybrid or novel heuristics of your own design
    # - aco (ant colony optimization)
    # - genetic algorithms
    # - k-opt
    # - tabu search

    # Example heuristic:
    # Randomly generate a permutation of cities.
    np.random.seed(42)  # Ensure reproducibility
    route = np.random.permutation(np.arange(len(_distances)))

    # Use local search to improve the route.
    route = funsearch.local_search(route, _distances)

    return route
