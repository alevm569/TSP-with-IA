import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining the nearest neighbor and cheapest insertion heuristics.
    """
    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(_distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        # Find the closest unvisited city
        closest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(closest_city)
        unvisited.remove(closest_city)

    # Use the cheapest insertion heuristic to refine the route
    for i in range(1, len(route)):
        for j in range(i + 1, len(route)):
            if _distances[route[i]][route[j]] > _distances[route[i]][route[j - 1]]:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
