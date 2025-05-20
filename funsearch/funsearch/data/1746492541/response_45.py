import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(0)  # Set random seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Nearest neighbor heuristic to generate an initial solution
    current_city = 0
    route = [current_city]
    visited = set([current_city])
    while len(visited) < len(_distances):
        next_city = np.argmin(_distances[current_city][[c for c in range(len(_distances)) if c not in visited]])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Local search to refine the solution
    for _ in range(100):
        i, j = np.random.randint(0, len(route), 2)
        route[i], route[j] = route[j], route[i]
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            route[i], route[j] = route[j], route[i]

    return tuple(route)
