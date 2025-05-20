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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a hybrid approach combining the nearest neighbor and 2-opt heuristics.
    """

    # Initial route using nearest neighbor heuristic
    route = np.arange(len(distances))
    current_city = 0
    for _ in range(len(route)):
        nearest_city = np.argmin(distances[current_city])
        route[_] = nearest_city
        current_city = nearest_city

    # Improve the route using 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = distances[route[i]][route[j]]
            distance_reversed = distances[route[i]][route[(j + 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
