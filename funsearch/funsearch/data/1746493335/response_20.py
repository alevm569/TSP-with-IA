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
    Improved version of find_best_route_v0 using a hybrid heuristic.

    Combines the nearest neighbor and 2-opt heuristics.
    """

    # Apply nearest neighbor to get an initial route
    current_city = 0
    route = [current_city]
    while len(route) < len(distances):
        nearest_city = np.argmin(distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Apply 2-opt to improve the route
    best_route = route[:]
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i:j+1] = new_route[j:i:-1]
            if calculate_route_distance(new_route, distances) < calculate_route_distance(best_route, distances):
                best_route = new_route

    return tuple(best_route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
