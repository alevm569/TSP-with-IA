import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Initialize random seed for reproducibility
    np.random.seed(42)

    # Use nearest neighbor heuristic to generate an initial route
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Perform local search to refine the route
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
