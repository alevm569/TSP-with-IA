import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



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
    # Implement a new heuristic or combine two or more heuristics.
    # For example, you could use a hybrid of nearest neighbor and cheapest insertion.

    # Set a seed for reproducibility.
    np.random.seed(42)

    # Generate a random initial route.
    num_cities = len(_distances)
    route = np.random.permutation(num_cities)

    # Perform local search to improve the route.
    for i in range(100):
        # Randomly swap two cities in the route.
        a, b = np.random.randint(num_cities, size=2)
        route[a], route[b] = route[b], route[a]

        # Calculate the distance of the improved route.
        distance = calculate_route_distance(route, _distances)

        # If the improved route is better, keep it.
        if distance < calculate_route_distance(route, _distances):
            route = route

    return route
