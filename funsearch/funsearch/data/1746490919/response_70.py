import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic that combines the nearest neighbor and cheapest insertion algorithms
    # Use the nearest neighbor algorithm to generate an initial route
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Use the cheapest insertion algorithm to iteratively add cities to the route
    while len(route) < len(_distances):
        # Find the city that is closest to the last city in the route
        nearest_city = np.argmin(_distances[route[-1]])

        # Ensure the city is not already in the route
        if nearest_city not in route:
            route.append(nearest_city)

    return tuple(route)
