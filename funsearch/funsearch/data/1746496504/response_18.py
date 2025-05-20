import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid heuristic that combines nearest neighbor and 2-opt
    route = funsearch.algorithms.hybrid(
        funsearch.algorithms.nearest_neighbor(_distances),
        funsearch.algorithms.two_opt(_distances),
    )

    # Perform local search to improve the route
    route = funsearch.algorithms.local_search(route, _distances)

    return route


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route
    best_route = funsearch.algorithms.genetic(
        _distances, population_size=100, generations=1000
    )

    return best_route
