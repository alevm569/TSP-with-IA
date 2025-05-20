import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining nearest neighbor, k-opt, and local search heuristics.
    """

    # Initialize the route using the nearest neighbor heuristic
    initial_route = funsearch.nearest_neighbor(_distances)

    # Apply k-opt to improve the route by swapping two randomly chosen cities
    best_route = funsearch.k_opt(initial_route, _distances)

    # Perform local search to refine the route by iteratively swapping two nearby cities
    best_route = funsearch.local_search(best_route, _distances)

    return best_route
