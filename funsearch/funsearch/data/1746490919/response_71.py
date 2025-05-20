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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.
    Uses a combination of nearest neighbor and local search heuristics.
    """

    # Initialize a random route
    num_cities = distances.shape[0]
    route = np.random.permutation(num_cities)

    # Nearest neighbor heuristic
    for i in range(num_cities):
        current_city = route[i]
        next_city = np.argmin(distances[current_city])
        route = np.insert(route, i + 1, next_city)

    # Local search
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
