import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Create an ACO optimizer
    optimizer = funsearch.ACOOptimizer(distance_matrix=_distances)

    # Run the optimization algorithm
    best_route = optimizer.optimize()

    return best_route
