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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Randomly initialize a route
    np.random.seed(42)
    route = np.random.permutation(np.arange(_distances.shape[0]))

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        i, j = np.random.randint(0, len(_distances), size=2)
        route = funsearch.two_opt_move(route, i, j)

    return route
