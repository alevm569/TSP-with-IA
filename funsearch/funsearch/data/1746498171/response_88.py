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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route using a hybrid approach.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function combines the nearest neighbor heuristic for initial route construction,
    cheapest insertion for route refinement, and local search for optimization.
    """

    # Nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Cheapest insertion to refine the route
    for _ in range(len(_distances)):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                if _distances[route[i]][route[j]] > _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i + 1) % len(route)]]:
                    route = route[:i] + route[j:j + 1] + route[i + 1:j] + route[i:i + 1]

    # Local search to optimize the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                if calculate_route_distance(route[:i] + route[j:i + 1] + route[i + 1:j] + route[j:], _distances) < calculate_route_distance(route, _distances):
                    route = route[:i] + route[j:i + 1] + route[i + 1:j] + route[j:]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
