import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(0)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a list of available cities
    available_cities = list(range(len(_distances)))

    # Start with a random city
    current_city = np.random.choice(available_cities)
    route = [current_city]

    # Remove the starting city from the list of available cities
    available_cities.remove(current_city)

    # Iterate until all cities have been visited
    while available_cities:
        # Find the nearest unvisited city
        nearest_city = min(available_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the list of available cities
        available_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
