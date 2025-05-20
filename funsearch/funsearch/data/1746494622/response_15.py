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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Use nearest neighbor to generate an initial route
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Use cheapest insertion to refine the route
    while len(route) < len(_distances):
        next_city = funsearch.cheapest_insertion(route, _distances)
        route.append(next_city)

    return tuple(route)
