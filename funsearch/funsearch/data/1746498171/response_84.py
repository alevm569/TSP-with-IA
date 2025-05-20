import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize a random route
    num_cities = len(_distances)
    route = np.random.permutation(num_cities)

    # Perform local search to find the best route
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]

            # Calculate the distance of the new route
            distance = calculate_route_distance(route, _distances)

            # If the new route is better, keep it
            if distance < calculate_route_distance(route, _distances):
                route = route

    return route
