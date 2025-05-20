import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Perform nearest neighbor search to initialize a route
    route = funsearch.nearest_neighbor(_distances)

    # Perform local search to improve the route
    route = funsearch.local_search(_distances, route)

    return route
