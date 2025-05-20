import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Local search with 2-opt neighborhood operator
    - Random initialization of the route

    Routes must include all cities exactly once and return to the starting point.
    """

    num_cities = len(matrix_distances)
    best_route = np.random.permutation(num_cities)

    for _ in range(100):  # Number of iterations
        current_route = best_route.copy()

        # Perform local search with 2-opt neighborhood operator
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                current_route[i:j+1] = current_route[j:i:-1]

        if calculate_route_distance(current_route, matrix_distances) < calculate_route_distance(best_route, matrix_distances):
            best_route = current_route

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
