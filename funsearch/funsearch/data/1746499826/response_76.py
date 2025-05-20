import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Create ACO optimizer object
    aco_optimizer = funsearch.ACOOptimizer(distance_matrix=_distances)

    # Run ACO optimization
    best_route = aco_optimizer.optimize()

    return best_route
