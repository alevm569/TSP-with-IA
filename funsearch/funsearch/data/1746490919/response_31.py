import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Create ACO optimizer object
    optimizer = funsearch.ACTOptimizer(distance_matrix=_distances)

    # Run ACO optimization
    best_route = optimizer.optimize()

    return best_route

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristic."""

    # Create hybrid optimizer object
    optimizer = funsearch.HybridOptimizer(distance_matrix=_distances)

    # Run hybrid optimization
    best_route = optimizer.optimize()

    return best_route
