import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Create an ACO optimizer object
    optimizer = funsearch.aco.AntColonyOptimizer(distances=_distances)

    # Run the optimizer to find the best route
    best_route = optimizer.optimize()

    # Return the best route as a tuple
    return best_route
