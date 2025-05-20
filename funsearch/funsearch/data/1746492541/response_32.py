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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Uses a hybrid heuristic combining nearest neighbor and 2-opt.
    """

    # Initialize the route using nearest neighbor
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    for _ in range(len(distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin([distance for distance in distances[current_city] if distance not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Perform 2-opt optimization to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_difference = distances[route[i]][route[j]] - distances[route[(i - 1) % len(route)]][route[(j + 1) % len(route)]]
            if distance_difference < 0:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
