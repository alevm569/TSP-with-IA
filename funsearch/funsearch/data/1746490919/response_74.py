import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use ACO algorithm with a custom heuristic
    def custom_heuristic(route: np.ndarray) -> float:
        # Calculate the total distance of the route, penalizing repeated or missing cities
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]

        # Penalty for repeated or missing cities
        for i in range(len(route)):
            if route.count(i) > 1:
                total_distance *= 1.5

        return total_distance

    aco = funsearch.aco.ACO(custom_heuristic)
    best_route = aco.solve(_distances)

    return best_route
