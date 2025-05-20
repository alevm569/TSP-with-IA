import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

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

    # Implement a hybrid heuristic that combines different strategies
    # For example, you could use the nearest neighbor strategy to generate an initial solution,
    # then apply the 2-opt heuristic to improve it iteratively.

    # Example hybrid heuristic using nearest neighbor and 2-opt
    route = np.random.permutation(len(_distances))
    while True:
        # Find the two cities that can be swapped to improve the route
        best_delta = float('inf')
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                delta = _distances[route[i]][route[(j + 1) % len(route)]] - _distances[route[i]][route[j]] - _distances[route[(j + 1) % len(route)]][route[i]]
                if delta < best_delta:
                    best_delta = delta
                    best_i = i
                    best_j = j

        # If no improvement is found, the route is optimal
        if best_delta == float('inf'):
            break

        # Swap the two cities and update the route
        route[best_i], route[best_j] = route[best_j], route[best_i]

    return route
