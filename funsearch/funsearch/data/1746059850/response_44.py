import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Implement a hybrid heuristic that combines multiple sub-heuristics, such as:
    # - Nearest neighbor to generate an initial route.
    # - Cheapest insertion to refine the route.
    # - Local search to improve the route by iteratively swapping nearby cities.

    # Example hybrid heuristic:
    # 1. Generate an initial route using nearest neighbor.
    # 2. Refine the route using cheapest insertion.
    # 3. Apply local search to optimize the route further.

    # Return the best route found by the hybrid heuristic.
    return tuple(best_route)
