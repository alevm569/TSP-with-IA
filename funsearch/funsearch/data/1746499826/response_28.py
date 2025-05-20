import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with ACO."""

    # Create an ACO optimizer object
    optimizer = funsearch.ACOOptimizer(distance_matrix=_distances)

    # Run the ACO optimization algorithm
    best_route = optimizer.optimize()

    return best_route
