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
    Improved version of `find_best_route_v2`.

    Uses a combination of local search and 2-opt heuristics.
    """

    # Generate an initial random route
    num_cities = len(distances)
    route = np.random.permutation(num_cities)

    # Local search: iteratively improve the route by swapping adjacent cities
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    # 2-opt heuristic: swap two adjacent cities and check if the route is improved
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            new_route = route.copy()
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                route = new_route

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
