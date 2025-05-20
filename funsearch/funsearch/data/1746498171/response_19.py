import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`.

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics,
    with a local search to refine the solution.
    """
    # Initialize the route using nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Apply 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if _distances[route[i]][route[j]] < _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i - 1) % len(route)]]:
                route[i:j+1] = route[j:i:-1]

    # Local search to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if _distances[route[i]][route[j]] < _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i - 1) % len(route)]]:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
