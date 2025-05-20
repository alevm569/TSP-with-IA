import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new version of the find_best_route function using a hybrid approach.
    # Combine different heuristics and strategies to improve performance.

    # Example hybrid approach using nearest neighbor and 2-opt:

    # 1. Initialize route using nearest neighbor heuristic.
    route = funsearch.nearest_neighbor(_distances)

    # 2. Apply 2-opt local search to improve the route.
    route = funsearch.two_opt(route, _distances)

    return route
