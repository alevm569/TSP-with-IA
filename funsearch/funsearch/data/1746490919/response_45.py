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
    Improved version of `find_best_route_v2`.

    Uses a combination of local search and 2-opt heuristics.
    """

    # Generate an initial random route
    num_cities = matrix_distances.shape[0]
    route = np.random.permutation(num_cities)

    # Local search with 2-opt neighborhood operator
    for _ in range(100):
        best_distance = calculate_route_distance(route, matrix_distances)

        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, matrix_distances)

                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return distance
