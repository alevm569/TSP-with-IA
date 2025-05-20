import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Create a set to track visited cities
    visited = set()

    # Start from the first city
    current_city = 0

    # Initialize the route list
    route = [current_city]

    # Iterate until all cities are visited
    while len(visited) < len(_distances):
        # Find the closest unvisited city
        min_distance = math.inf
        for i in range(len(_distances)):
            if i not in visited and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        visited.add(next_city)

        # Update the current city
        current_city = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
