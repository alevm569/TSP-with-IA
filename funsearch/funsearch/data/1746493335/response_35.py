import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristics."""

    # Use a combination of nearest neighbor and cheapest insertion heuristics
    route = funsearch.nearest_neighbor(_distances)
    funsearch.cheapest_insertion(route, _distances)

    # Perform local search to improve the route
    funsearch.local_search(route, _distances)

    return route
