import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Initialize a permutation of cities
    cities = np.arange(len(_distances))
    np.random.shuffle(cities)

    # Use a combination of nearest neighbor and cheapest insertion heuristics
    current_city = cities[0]
    route = [current_city]

    while len(route) < len(cities):
        next_city = funsearch.nearest_neighbor(current_city, route, _distances)
        if next_city not in route:
            route.append(next_city)
            current_city = next_city
        else:
            next_city = funsearch.cheapest_insertion(current_city, route, _distances)
            route.append(next_city)
            current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
