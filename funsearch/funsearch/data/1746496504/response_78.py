import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Args:
        distances: A square matrix of distances between cities.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(distances):
        current_city = route[-1]
        nearest_city = np.argmin(distances[current_city][~np.isin(np.arange(len(distances)), visited)])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Local search heuristic
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
