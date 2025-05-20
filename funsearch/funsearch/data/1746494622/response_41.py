import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with new heuristics."""

    # Use a hybrid approach combining local search with 2-opt
    route = funsearch.local_search(funsearch.two_opt, _distances)

    # Ensure all cities are included and the route returns to the starting city
    assert len(set(route)) == len(_distances)
    assert route[0] == route[-1]

    return route
