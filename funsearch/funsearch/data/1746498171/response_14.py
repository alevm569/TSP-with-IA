import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Create an ACO optimizer
    optimizer = funsearch.aco.AntColonyOptimizer(distances=_distances)

    # Run the optimizer to find the best route
    best_route = optimizer.optimize()

    return best_route
