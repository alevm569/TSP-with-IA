import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""

    # Create an ACO optimizer
    optimizer = funsearch.aco.ACTOptimizer(distances=_distances)

    # Run the ACO algorithm
    best_route = optimizer.optimize()

    return best_route
