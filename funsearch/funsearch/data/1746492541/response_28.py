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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic combining nearest neighbor and k-opt.
    """

    # Initialize a random permutation of cities
    np.random.seed(42)
    initial_route = np.random.permutation(len(_distances))

    # Use nearest neighbor to find a good initial route
    current_route = nearest_neighbor(_distances, initial_route[0])

    # Perform k-opt to refine the route
    for _ in range(10):
        current_route = k_opt(_distances, current_route)

    return current_route
