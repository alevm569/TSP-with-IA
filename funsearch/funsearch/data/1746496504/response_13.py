import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with additional heuristics."""

    # Use hybrid heuristic combining nearest neighbor and 2-opt
    start_city = 0  # Choose starting city arbitrarily
    route = funsearch.nearest_neighbor(start_city, _distances)
    route = funsearch.two_opt(route, _distances)

    return route
