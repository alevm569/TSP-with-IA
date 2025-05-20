import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid heuristic combining nearest neighbor and 2-opt
    best_route = funsearch.hybrid_heuristic(
        _distances,
        funsearch.nearest_neighbor,
        funsearch.two_opt,
        max_iterations=1000,
        step_size=10
    )

    return best_route
