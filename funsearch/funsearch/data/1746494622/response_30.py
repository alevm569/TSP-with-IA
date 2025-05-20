import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid approach combining two heuristics:
    # 1. Nearest neighbor to generate an initial route.
    # 2. 2-opt local search to improve the route by swapping two cities.

    # Generate an initial route using the nearest neighbor heuristic
    n = len(_distances)
    current_city = np.random.randint(n)
    route = [current_city]
    visited = np.zeros(n, dtype=bool)
    visited[current_city] = True

    while not np.all(visited):
        nearest_city = np.argmin(_distances[current_city][~visited])
        route.append(nearest_city)
        visited[nearest_city] = True
        current_city = nearest_city

    # Perform local search using the 2-opt heuristic to improve the route
    for _ in range(100):
        i, j = np.random.randint(n, size=2)
        if calculate_route_distance(route, _distances) > calculate_route_distance(route[:i] + route[i+j:] + route[i:i+j], _distances):
            route = route[:i] + route[i+j:] + route[i:i+j]

    return tuple(route)
