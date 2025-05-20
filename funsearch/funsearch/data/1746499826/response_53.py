import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Use nearest neighbor heuristic to generate an initial solution
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    unvisited_cities = list(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Use 2-opt heuristic to improve the solution
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_before = _distances[route[i]][route[j]]
                distance_after = _distances[route[i]][route[j - 1]] + _distances[route[j]][route[i]]
                if distance_after < distance_before:
                    route[i:j] = route[j-1:i:-1]

    return tuple(route)
