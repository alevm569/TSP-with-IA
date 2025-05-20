import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the route with the first city
    route = [0]

    # Use a set to keep track of visited cities
    visited = {0}

    # Repeat until all cities are visited
    while len(visited) < len(_distances):

        # Find the closest unvisited city to the last city in the route
        best_city = None
        best_distance = math.inf
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[route[-1]][i]
                if distance < best_distance:
                    best_city = i
                    best_distance = distance

        # Add the best city to the route and mark it as visited
        route.append(best_city)
        visited.add(best_city)

    # Return the route by converting it to a tuple
    return tuple(route)
