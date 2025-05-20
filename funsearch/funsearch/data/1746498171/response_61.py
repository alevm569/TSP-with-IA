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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 with hybrid heuristic.

    Hybrid heuristic combines:
        - Nearest neighbor
        - Cheapest insertion
        - 2-opt local search

    """
    # Nearest neighbor heuristic
    current_city = 0
    route = [current_city]

    for _ in range(len(matrix_distances) - 1):
        nearest_city = np.argmin(matrix_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Cheapest insertion heuristic
    for i in range(1, len(matrix_distances)):
        best_distance = float('inf')
        best_city = None

        for j in range(len(matrix_distances)):
            if j not in route:
                distance = matrix_distances[route[-1]][j]
                if distance < best_distance:
                    best_distance = distance
                    best_city = j

        route.append(best_city)

    # 2-opt local search
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_before = calculate_route_distance(route, matrix_distances)
            route[i], route[j] = route[j], route[i]
            distance_after = calculate_route_distance(route, matrix_distances)

            if distance_after < distance_before:
                break
            else:
                route[i], route[j] = route[j], route[i]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0

    for i in range(len(route)):
        j = (i + 1) % len(route)
        total_distance += matrix_distances[route[i]][route[j]]

    return total_distance
