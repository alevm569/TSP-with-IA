import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Initialize starting city
    start_city = 0

    # Create a list of unvisited cities
    unvisited_cities = list(range(1, len(_distances)))

    # Initialize the best route with the starting city
    best_route = [start_city]

    # Iterate until all cities are visited
    while unvisited_cities:
        # Get the current city
        current_city = best_route[-1]

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        best_route.append(nearest_city)

        # Remove the nearest city from the list of unvisited cities
        unvisited_cities.remove(nearest_city)

    # Add the starting city back to the route
    best_route.append(start_city)

    return tuple(best_route)
