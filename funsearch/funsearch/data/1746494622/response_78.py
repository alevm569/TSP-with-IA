import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a set of unvisited cities
    unvisited = set(range(len(_distances)))

    # Start from the first city
    current_city = 0
    route = [current_city]

    # Iterate until all cities have been visited
    while len(unvisited) > 0:

        # Find the nearest unvisited city
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
