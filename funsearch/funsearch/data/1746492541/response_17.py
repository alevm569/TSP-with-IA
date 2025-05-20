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
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining nearest neighbor and k-opt heuristics.
    """

    # Nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(distances):
        nearest_city = np.argmin(distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # K-opt heuristic for local search
    for k in range(2, len(distances)):
        for i in range(len(distances)):
            for j in range(i + 1, len(distances)):
                for l in range(j + 1, len(distances)):
                    new_route = route.copy()
                    new_route[i:l+1] = route[l:i-1:-1]
                    if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                        route = new_route

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
