import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Initialize ACO algorithm
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities)

    # Run ACO algorithm
    best_route = aco.run()

    # Convert ACO route to a tuple of integers
    return tuple(best_route)
