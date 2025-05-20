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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Implement a hybrid heuristic combining different techniques
    # For example, use nearest neighbor to generate an initial solution, then apply a local search algorithm (e.g., 2-opt) to improve it.

    # Example of a hybrid heuristic:
    # 1. Generate an initial route using the nearest neighbor heuristic.
    # 2. Apply the 2-opt local search algorithm to the initial route.
    # 3. Repeat step 2 until no further improvements are made.

    # Return the best route found
    return best_route
