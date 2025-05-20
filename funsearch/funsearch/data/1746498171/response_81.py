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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines multiple strategies
    # For example, you could use a combination of the nearest neighbor and cheapest insertion heuristics

    # Example hybrid heuristic using nearest neighbor and cheapest insertion
    route = np.random.permutation(len(matrix_distances))
    while len(route) < len(matrix_distances):
        current_city = route[-1]
        nearest_city = np.argmin(matrix_distances[current_city])
        cheapest_city = np.argmin(matrix_distances[route[-1]])
        if nearest_city not in route and cheapest_city not in route:
            route = np.append(route, nearest_city)
        else:
            route = np.append(route, cheapest_city)

    return route.astype(int)
