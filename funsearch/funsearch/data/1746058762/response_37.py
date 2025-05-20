import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a combination of local search and a 2-opt heuristic.
    best_route = funsearch.local_search(funsearch.two_opt, _distances)

    return best_route

def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3`."""

    # Use a hybrid heuristic that combines local search with a 2-opt heuristic and a nearest neighbor heuristic.
    best_route = funsearch.hybrid_heuristic(funsearch.local_search, funsearch.two_opt, funsearch.nearest_neighbor, _distances)

    return best_route

def find_best_route_v5(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v4`."""

    # Use a genetic algorithm to find the best route.
    best_route = funsearch.genetic_algorithm(_distances)

    return best_route
