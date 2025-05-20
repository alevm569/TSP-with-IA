import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create an initial route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(_distances):
        current_city = route[-1]
        nearest_city = np.argmin([_distances[current_city][city] for city in range(len(_distances)) if city not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Perform local search to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
