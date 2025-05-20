import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines multiple heuristics
    # For example, you could use a combination of nearest neighbor and 2-opt

    # Generate an initial solution using nearest neighbor
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        nearest_city = np.argmin(_distances[current_city][~np.isin(_distances[current_city], visited)])
        route.append(nearest_city)
        visited.add(nearest_city)
        current_city = nearest_city

    # Improve the solution using 2-opt
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_diff = _distances[route[i]][route[j]] - _distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]]
            if distance_diff < 0:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
