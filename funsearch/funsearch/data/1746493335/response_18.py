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
    optimizer = funsearch.aco.ACOOptimizer(
        distance_matrix=_distances,
        num_ants=100,
        num_iterations=1000,
        alpha=1.0,
        beta=2.0,
        rho=0.1,
    )

    # Run the optimization
    best_route = optimizer.optimize()

    return best_route
