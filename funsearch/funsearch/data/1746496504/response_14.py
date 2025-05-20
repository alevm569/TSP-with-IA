import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with additional heuristics."""

    # Use ACO (ant colony optimization)
    num_ants = 10
    num_iterations = 100
    aco = funsearch.AntColonyOptimization(_distances, num_ants, num_iterations)

    # Run ACO and get the best route
    best_route = aco.run()

    return best_route
