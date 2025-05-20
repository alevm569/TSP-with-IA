import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Generate random initial route
    np.random.seed(42)  # Set seed for reproducibility
    random_route = np.random.permutation(len(_distances))

    # Use a combination of local search and 2-opt heuristics
    best_route = local_search(random_route, _distances)
    best_route = two_opt(best_route, _distances)

    return best_route

def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    # Perform local search to improve the route
    # ...
    return improved_route

def two_opt(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    # Perform 2-opt heuristic to improve the route
    # ...
    return improved_route
