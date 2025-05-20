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
    """Improved version of `find_best_route_v2` with 2-opt heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform 2-opt local search
    for _ in range(100):
        # Randomly select two segments of the route
        i, j = np.random.randint(0, len(_distances), 2)

        # Apply the 2-opt swap
        route[i:j] = route[j-1:i-1:-1]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the 2-opt swap
            route[i:j] = route[j-1:i-1:-1]

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
