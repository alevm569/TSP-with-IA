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
    num_cities = len(matrix_distances)

    # Initialize a random route.
    current_route = np.random.permutation(num_cities)

    # Perform local search to find a better route.
    for i in range(100):
        # Randomly swap two cities in the route.
        city1, city2 = np.random.randint(num_cities, size=2)
        current_route[city1], current_route[city2] = current_route[city2], current_route[city1]

        # Calculate the distance of the new route.
        new_distance = calculate_route_distance(current_route, matrix_distances)

        # If the new route is better, keep it.
        if new_distance < calculate_route_distance(current_route, matrix_distances):
            current_route = new_route

    return current_route

def calculate_route_distance(route: np.ndarray, matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
