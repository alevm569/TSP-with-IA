import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with new heuristics and randomness stability."""

    # Use ACO (Ant Colony Optimization)
    best_route = funsearch.aco.solve(_distances)

    # Return the best route as a tuple
    return best_route
