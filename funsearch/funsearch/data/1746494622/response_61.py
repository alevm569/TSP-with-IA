import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Initialize the ACO solver with appropriate parameters
    aco_solver = funsearch.ACSolver(len(_distances), alpha=1.0, beta=2.0, rho=0.1)

    # Run the ACO solver to find the best route
    best_route = aco_solver.solve(_distances)

    # Return the best route as a tuple
    return tuple(best_route)
