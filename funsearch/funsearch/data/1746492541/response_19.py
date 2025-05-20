import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid approach combining nearest neighbor and local search heuristics.
    """

    # Initialize current route and distance
    current_route = np.random.permutation(len(_distances))
    current_distance = calculate_route_distance(current_route, _distances)

    # Local search iterations
    for _ in range(100):
        # Generate a new route by swapping two random cities
        new_route = current_route.copy()
        i, j = np.random.randint(0, len(_distances), size=2)
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate distance for the new route
        new_distance = calculate_route_distance(new_route, _distances)

        # If the new route is better, update the current route and distance
        if new_distance < current_distance:
            current_route = new_route
            current_distance = new_distance

    return current_route
