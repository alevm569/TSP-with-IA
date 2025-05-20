import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


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
    Improved version of find_best_route_v2.

    Uses a hybrid heuristic that combines the nearest neighbor and local search heuristics.
    """

    # Generate an initial route using nearest neighbor.
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        current_city = route[-1]
        closest_city = min(remaining_cities, key=lambda c: matrix_distances[current_city][c])
        route.append(closest_city)
        remaining_cities.remove(closest_city)

    # Perform local search to improve the route.
    for _ in range(100):
        i, j = np.random.randint(0, len(route), size=2)
        route[i], route[j] = route[j], route[i]

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    assert len(set(route)) == len(matrix_distances)
    assert route[0] == start_city

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
