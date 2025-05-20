import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with a hybrid heuristic."""

    # Use a combination of nearest neighbor and 2-opt heuristics
    route = funsearch.nearest_neighbor(_distances)
    route = funsearch.two_opt(route, _distances)

    return route
