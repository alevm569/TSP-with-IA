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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the best route and its distance
    best_route = np.arange(len(matrix_distances))
    best_distance = np.inf

    # Generate random permutations of the cities
    np.random.seed(42)  # Set a seed for reproducibility
    for _ in range(1000):  # Run multiple iterations
        route = np.random.permutation(len(matrix_distances))

        # Calculate the total distance of the route
        distance = 0
        for i in range(len(route)):
            distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

        # Update the best route if necessary
        if distance < best_distance:
            best_distance = distance
            best_route = route

    return best_route
