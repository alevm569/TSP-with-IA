import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
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
    Finds the best route using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Initialize a random route
    num_cities = distances.shape[0]
    route = np.random.permutation(num_cities)

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        # Randomly choose two cities in the route
        i, j = np.random.randint(num_cities, size=2)

        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            pass
        else:
            # Reverse the two cities in the route
            route[i:j+1] = route[j:i:-1]

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]

    return total_distance
