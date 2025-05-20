import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with ACO heuristic."""

    # Create an ACO optimizer
    optimizer = funsearch.ACOOptimizer(
        distance_matrix=_distances,
        num_ants=10,
        max_iterations=100,
        alpha=1.0,  # Weighting factor for pheromone influence
        beta=2.0,  # Weighting factor for distance influence
        rho=0.1,  # Pheromone decay rate
    )

    # Run the ACO optimizer
    best_route = optimizer.optimize()

    # Return the best route
    return best_route
