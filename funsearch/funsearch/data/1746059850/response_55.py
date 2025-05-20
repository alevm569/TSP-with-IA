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


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply 2-opt local search
    best_route = find_best_route_v2(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 2, len(best_route)):
                new_route = two_opt(best_route, i, j)
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route
                    improved = True

        if not improved:
            break

    return best_route


def two_opt(route: tuple[int, ...], i: int, j: int) -> tuple[int, ...]:
    """Perform the 2-opt move on a route."""
    return route[:i] + route[j:i:-1] + route[j + 1:]
