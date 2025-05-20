import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Initialize random seed for reproducibility
    np.random.seed(42)

    # Use ACO heuristic to generate initial route
    aco = funsearch.ACO(_distances)
    initial_route = aco.run()

    # Apply local search to refine the route
    ls = funsearch.LocalSearch(initial_route, _distances)
    best_route = ls.run()

    # Return the best route as a tuple
    return best_route
