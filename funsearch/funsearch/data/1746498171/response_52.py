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
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining nearest neighbor, cheapest insertion, and local search.
    """
    # Initial route using nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(distances):
        nearest_city = np.argmin(distances[current_city][~np.isin(np.arange(len(distances)), visited)])
        route.append(nearest_city)
        visited.add(nearest_city)
        current_city = nearest_city

    # Optimize route using local search
    for _ in range(100):
        route = local_search(route, distances)

    return tuple(route)

def local_search(route: list[int], distances: np.ndarray) -> list[int]:
    """Performs local search to improve a given route."""
    best_route = route[:]
    best_distance = calculate_route_distance(route, distances)

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]
            distance = calculate_route_distance(route, distances)

            if distance < best_distance:
                best_route = route[:]
                best_distance = distance

            # Restore the original route
            route[i], route[j] = route[j], route[i]

    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
