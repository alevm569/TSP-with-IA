import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Create an ACO object with the distance matrix
    aco = funsearch.ACO(_distances)

    # Run the ACO algorithm for 100 iterations
    best_route = aco.run(iterations=100)

    # Return the best route
    return best_route
