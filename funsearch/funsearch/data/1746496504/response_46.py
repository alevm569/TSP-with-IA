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


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using local search."""

    # Generate an initial random route
    current_route = np.random.permutation(len(_distances))

    # Perform local search
    while True:
        best_distance = calculate_route_distance(current_route, _distances)

        # Try swapping two random cities in the route
        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                new_route = current_route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is shorter, keep it
                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route

        # If no improvements are found, stop the search
        if best_distance == calculate_route_distance(current_route, _distances):
            break

    return current_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
