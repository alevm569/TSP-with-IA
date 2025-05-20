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

    best_route = find_best_route_v3(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic or combine two or more heuristics.
    # Use local search to optimize the route.

    # Example of a new heuristic:
    # - Start from an arbitrary city.
    # - Iterate through the remaining cities in order of their distance from the current city.
    # - Add the closest city to the route and remove it from the candidate set.
    # - Repeat until all cities have been visited.

    # Example of local search:
    # - Randomly swap two cities in the route.
    # - If the new route is better, keep it.
    # - Otherwise, revert to the previous route.

    # ...

    # Return the best route found.
    return tuple(best_route)
