import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new version of the find_best_route function
    # Use a hybrid heuristic that combines two or more of the following:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - hybrid or novel heuristics of your own design
    # - aco (ant colony optimization)
    # - genetic algorithms
    # - k-opt
    # - tabu search

    # Ensure reproducibility by setting a seed
    np.random.seed(42)

    # Initialize the route
    route = np.random.permutation(np.arange(len(_distances)))

    # Perform local search to improve the route
    for _ in range(100):
        i, j = np.random.randint(0, len(_distances), 2)
        route = funsearch.local_search(route, _distances, i, j)

    # Return the best route
    return route
