import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Use a combination of local search and 2-opt heuristics
    initial_route = np.random.permutation(len(_distances))
    best_route = funsearch.localsearch.greedy_localsearch(initial_route, _distances)
    best_route = funsearch.optimize.two_opt(best_route, _distances)

    return best_route
