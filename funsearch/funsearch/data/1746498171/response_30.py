import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""

    # Create an ACO optimizer
    optimizer = funsearch.aco.ACTOptimizer(
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
        number_ants=10,
        iterations=100,
    )

    # Run the ACO optimization
    best_route = optimizer.optimize()

    return best_route
