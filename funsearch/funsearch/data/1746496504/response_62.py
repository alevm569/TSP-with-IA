import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    np.random.seed(42)  # Set a seed for reproducibility
    aco = funsearch.ACO(distance_matrix=_distances)
    best_route = aco.run()
    return best_route
