import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Create an ACO optimizer object
    optimizer = funsearch.ACOOptimizer(distance_matrix=_distances)

    # Run the optimization algorithm
    best_route = optimizer.optimize()

    # Return the best route as a tuple
    return best_route
