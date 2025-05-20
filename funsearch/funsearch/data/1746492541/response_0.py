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


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic combining two or more of the following:
    # - Nearest neighbor
    # - Cheapest insertion
    # - Local search
    # - 2-opt
    # - ACO (ant colony optimization)
    # - Genetic algorithms
    # - K-opt
    # - Tabu search

    # Example hybrid heuristic:
    # 1. Start with a random route.
    # 2. Apply the 2-opt heuristic to improve the route.
    # 3. Use ACO to guide the search, directing it towards promising neighborhoods.

    # Return the best route found.
    return tuple(best_route)
