import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(_distances)))

    # Start with a random city
    current_city = np.random.choice(unvisited_cities)
    route = [current_city]

    # Visit all unvisited cities
    while unvisited_cities:
        # Find the nearest unvisited city
        next_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        unvisited_cities.remove(next_city)

        # Add the next city to the route
        route.append(next_city)

        # Update the current city
        current_city = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
