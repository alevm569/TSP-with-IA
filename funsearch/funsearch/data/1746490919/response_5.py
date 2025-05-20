import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    # Initialize a random starting point
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Find the nearest neighbor for each city
    for _ in range(len(_distances) - 1):
        nearest_city = np.argmin(_distances[current_city])
        route.append(nearest_city)
        current_city = nearest_city

    # Apply 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i:j][::-1] + route[j:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
