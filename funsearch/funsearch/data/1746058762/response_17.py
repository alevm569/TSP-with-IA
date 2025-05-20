import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or hybrid of heuristics here.
    # Consider using genetic algorithms, metaheuristics, or other advanced techniques.

    # Example heuristic:
    # - Start from an arbitrary city.
    # - Find the nearest unvisited city.
    # - Repeat until all cities have been visited.
    # - Return to the starting city.

    # Initialize starting city
    current_city = 0
    visited = set([current_city])
    route = [current_city]

    # Find the remaining unvisited cities
    remaining_cities = set(range(len(_distances))) - visited

    # Iterate until all cities are visited
    while remaining_cities:
        # Find the nearest unvisited city
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        visited.add(nearest_city)
        remaining_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
