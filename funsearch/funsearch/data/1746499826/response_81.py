import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with additional heuristics."""

    # Perform simulated annealing to find a close-to-optimal solution
    best_route = funsearch.simulated_annealing(_distances)

    # Perform local search to refine the solution
    best_route = funsearch.local_search(best_route, _distances)

    return best_route
