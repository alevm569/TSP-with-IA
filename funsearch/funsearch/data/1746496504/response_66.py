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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    num_cities = distances.shape[0]
    route = np.random.permutation(num_cities)

    # Perform local search to optimize the route
    for _ in range(100):
        best_distance = calculate_route_distance(route, distances)

        # Randomly swap two cities in the route
        i, j = np.random.randint(num_cities, size=2)
        route[i], route[j] = route[j], route[i]

        # Calculate the distance of the modified route
        distance = calculate_route_distance(route, distances)

        # Keep the route with the lower distance
        if distance < best_distance:
            best_distance = distance
        else:
            route[i], route[j] = route[j], route[i]

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
