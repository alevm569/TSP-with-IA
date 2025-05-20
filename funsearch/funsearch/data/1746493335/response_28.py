import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Create an ACO optimizer object
    optimizer = funsearch.ACOOptimizer()

    # Create an ACO problem object
    problem = funsearch.TSPProblem(_distances)

    # Optimize the route using ACO
    best_route = optimizer.optimize(problem)

    return best_route
