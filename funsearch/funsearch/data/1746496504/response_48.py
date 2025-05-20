import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of `find_best_route_v0` using a hybrid approach.

    - Starts with a random permutation of cities.
    - Uses a 2-opt heuristic to iteratively swap two cities in the route to find a shorter distance.
    - Repeats the 2-opt heuristic until no further improvements are found.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The best valid route as a permutation of city indices.
    """

    # Initialize a random route
    num_cities = len(matrix_distances)
    route = np.random.permutation(num_cities)

    # Perform 2-opt heuristic until convergence
    best_distance = calculate_route_distance(route, matrix_distances)
    while True:
        improved = False
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                distance_diff = matrix_distances[route[i]][route[j]] + \
                                matrix_distances[route[(j + 1) % num_cities]][route[(i + 1) % num_cities]] - \
                                matrix_distances[route[i]][route[(j + 1) % num_cities]] - \
                                matrix_distances[route[j]][route[(i + 1) % num_cities]]

                if distance_diff < 0:
                    route[i:j+1] = route[j:i:-1]
                    improved = True

        if not improved:
            break

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
