import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristics."""

    # Use a combination of local search and 2-opt heuristics
    best_route = funsearch.local_search(funsearch.two_opt, _distances)

    # Perform a round of simulated annealing to improve the solution
    best_route = funsearch.simulated_annealing(funsearch.two_opt, _distances, best_route)

    return best_route
