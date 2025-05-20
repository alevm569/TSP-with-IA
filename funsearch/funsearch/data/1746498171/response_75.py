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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    num_cities = len(distances)
    route = np.random.permutation(num_cities)

    # Local search with 2-opt move operator
    for _ in range(100):  # Number of iterations
        # Randomly select two cities
        i, j = np.random.randint(num_cities, size=2)

        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

        # Check if the new route is shorter
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            pass  # Keep the new route
        else:
            # Restore the original route
            route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
