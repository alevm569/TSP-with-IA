import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Define a new heuristic function
def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use a nearest neighbor heuristic to generate an initial route
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        min_distance = np.inf
        next_city = None

        # Find the city with the minimum distance from the current city
        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route
        route.append(next_city)
        current_city = next_city

    # Add the starting city to the end of the route
    route.append(start_city)

    return tuple(route)
