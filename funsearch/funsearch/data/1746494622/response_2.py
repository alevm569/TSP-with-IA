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
    """Improved version of `find_best_route_v1` with the following improvements:**

    - Uses a hybrid heuristic combining nearest neighbor and cheapest insertion.
    - Implements a local search strategy to improve the initial solution.
    """

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(distances):
        next_city = np.argmin(distances[current_city])
        if next_city not in route:
            route.append(next_city)
            current_city = next_city

    # Use the cheapest insertion heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if distances[route[i]][route[j]] > distances[route[i]][route[(j + 1) % len(route)]]:
                route[i], route[j] = route[j], route[i]

    # Perform local search to improve the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
