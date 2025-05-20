import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Use local search to refine the route
    for i in range(100):  # Number of iterations
        for j in range(len(route)):
            for k in range(j + 2, len(route)):
                new_route = route[:]
                new_route[j:k] = new_route[k:j:-1]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return tuple(route)
