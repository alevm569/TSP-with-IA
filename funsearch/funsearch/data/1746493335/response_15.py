import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random permutation of cities
    initial_route = np.random.permutation(len(_distances))

    # Use nearest neighbor to find the initial route
    current_city = initial_route[0]
    route = [current_city]

    # Use cheapest insertion to fill in the remaining cities
    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    return tuple(route)
