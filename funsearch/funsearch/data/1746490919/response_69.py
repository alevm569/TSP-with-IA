import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic that combines two or more of the following:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - hybrid or novel heuristics of your own design

    # Example hybrid heuristic:
    # 1. Use nearest neighbor to generate an initial route.
    # 2. Perform local search to improve the route.

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    # Return the best route found.

    # Initialize a set of unvisited cities
    unvisited_cities = set(range(len(_distances)))

    # Start from the first city
    current_city = 0

    # Initialize the route
    route = [current_city]

    # Iterate until all cities have been visited
    while len(unvisited_cities) > 0:
        # Remove the current city from the set of unvisited cities
        unvisited_cities.remove(current_city)

        # Find the city with the shortest distance to the current city
        best_city = None
        best_distance = float('inf')
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < best_distance:
                best_city = city
                best_distance = distance

        # Add the best city to the route
        route.append(best_city)

        # Set the current city to the best city
        current_city = best_city

    # Return the route
    return tuple(route)
