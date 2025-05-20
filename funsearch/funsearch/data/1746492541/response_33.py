import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Create ACO object
    aco = funsearch.ACO(len(_distances))

    # Run ACO algorithm
    best_route, best_distance = aco.solve(_distances)

    return best_route
