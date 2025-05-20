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
    Improved version of find_best_route_v0.

    Uses a hybrid heuristic combining nearest neighbor and k-opt.
    """

    # Use nearest neighbor to generate an initial route
    current_city = np.random.randint(len(distances))
    route = [current_city]

    while len(route) < len(distances):
        next_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in route])
        route.append(next_city)
        current_city = next_city

    # Use k-opt to improve the route
    for k in range(2, len(distances)):
        best_route = route
        for i in range(len(route)):
            for j in range(i + k, len(route)):
                new_route = route[:i] + route[i+j-k+1:j+1] + route[i:i+j-k+1] + route[j+1:]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(best_route, distances):
                    best_route = new_route
        route = best_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
