import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform local search on a route generated using the nearest neighbor heuristic
    best_route = funsearch.local_search(
        funsearch.nearest_neighbor(_distances),
        funsearch.tabu_search(_distances),
        max_iterations=1000
    )

    return best_route
