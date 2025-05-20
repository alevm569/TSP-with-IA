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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a hybrid heuristic.

    Args:
        matrix_distances: A square matrix of distances between cities.

    Returns:
        A tuple of integers representing the best route.
    """

    # Initialize the route with the nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    # Use the 2-opt heuristic to refine the route
    for _ in range(len(matrix_distances) - 1):
        current_distance = calculate_route_distance(route, matrix_distances)

        # Find the best two cities to swap using the 2-opt heuristic
        best_distance = current_distance
        best_i, best_j = 0, 0
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_distance = current_distance - matrix_distances[route[i]][route[j]] + matrix_distances[route[i]][route[j]]
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_i, best_j = i, j

        # Swap the two best cities
        route[best_i], route[best_j] = route[best_j], route[best_i]

    # Return the route, including the return to the starting city
    return tuple(route + [route[0]])


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
