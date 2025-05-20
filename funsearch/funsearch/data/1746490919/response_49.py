import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Create an ACO object
    aco = funsearch.ACO(distance_matrix=_distances, population_size=100, iterations=100)

    # Run the ACO algorithm
    aco.run()

    # Get the best route found by ACO
    best_route = aco.best_route

    return best_route
