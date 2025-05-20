import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid heuristic that combines nearest neighbor and cheapest insertion
    initial_route = funsearch.nearest_neighbor(_distances)
    best_route = funsearch.cheapest_insertion(initial_route, _distances)

    # Perform local search to refine the route
    best_route = funsearch.local_search(best_route, _distances)

    return best_route
