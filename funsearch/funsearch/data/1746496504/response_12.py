import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO heuristic."""

    # Initialize ACO algorithm
    aco = funsearch.ACO(distance_matrix=_distances)

    # Run ACO algorithm to find the best route
    best_route = aco.run()

    # Return the best route as a tuple
    return best_route
