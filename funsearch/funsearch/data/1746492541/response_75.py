import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = _distances[route[i]][route[j]] + _distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]]
            distance_reversed = _distances[route[i]][route[(j - 1) % len(route)]] + _distances[route[j]][route[(j + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
