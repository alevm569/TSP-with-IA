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
    # Implement your new heuristics here.

    # Example using a hybrid approach with nearest neighbor and 2-opt:
    # 1. Find the nearest neighbor of the starting city.
    # 2. Repeatedly find the nearest neighbor of the current city and add it to the route.
    # 3. Perform 2-opt swaps to improve the route quality.

    # Return the best route found.
    return tuple(best_route)
