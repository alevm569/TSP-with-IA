import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with additional heuristics."""

    # Use a hybrid heuristic combining nearest neighbor and 2-opt
    # Start with nearest neighbor to build an initial route
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    unvisited = set(range(len(_distances))) - {start_city}

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Use 2-opt to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j+1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
