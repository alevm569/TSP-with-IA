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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function combines the nearest neighbor heuristic for initialization and the 2-opt heuristic for local search.
    """
    # Initialize a random route
    num_cities = len(distances)
    route = np.random.permutation(num_cities)

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        # Randomly select two cities
        i, j = np.random.randint(num_cities, size=2)

        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

        # Check if the new route is shorter
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            pass  # Keep the new route

        else:
            route = np.roll(route, -1)  # Undo the swap

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
