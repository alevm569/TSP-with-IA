import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Initialize ACO algorithm
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities)

    # Run ACO algorithm for 100 iterations
    best_route = aco.run(100)

    # Return the best route found
    return best_route
