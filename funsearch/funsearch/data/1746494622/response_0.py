import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(0)

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

    # Implement a hybrid heuristic using nearest neighbor and 2-opt
    # Generate an initial route using nearest neighbor
    start_city = 0
    route = [start_city]
    while len(route) < len(_distances):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)

    # Apply 2-opt to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_before = _distances[route[i]][route[j]]
            route[i], route[j] = route[j], route[i]
            distance_after = _distances[route[i]][route[j]]
            if distance_after < distance_before:
                break
            route[i], route[j] = route[j], route[i]

    return tuple(route)
