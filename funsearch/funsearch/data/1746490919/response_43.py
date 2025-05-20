import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Combine nearest neighbor and 2-opt heuristics
    initial_route = funsearch.nearest_neighbor(_distances)
    improved_route = funsearch.two_opt(initial_route, _distances)

    # Perform local search to further refine the route
    optimized_route = funsearch.local_search(improved_route, _distances)

    return optimized_route
