import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor,
        funsearch.two_opt,
        distance_matrix=_distances,
        iterations=100,
    )

    return best_route
