import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities, _distances)

    # Run ACO for 100 iterations
    best_route = aco.solve(100)

    return best_route
