import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a combination of nearest neighbor and 2-opt heuristics
    route = funsearch.nearest_neighbor(_distances)
    funsearch.two_opt(route, _distances)

    return route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use genetic algorithms to find a good initial route
    initial_route = funsearch.genetic_algorithm(_distances)

    # Use local search to refine the route
    best_route = funsearch.local_search(initial_route, _distances)

    return best_route
