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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid Heuristic:
        - Use nearest neighbor to generate an initial route.
        - Perform local search with the 2-opt heuristic to refine the route.

    Routes must include all cities exactly once and return to the starting point.
    """
    # Generate an initial route using nearest neighbor
    start_city = 0
    route = [start_city]
    unvisited_cities = list(range(1, len(matrix_distances)))

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Perform local search with 2-opt heuristic
    for _ in range(100):  # Number of iterations for local search
        best_distance = calculate_route_distance(route, matrix_distances)

        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                reversed_route = route[:i] + route[i:j][::-1] + route[j:]
                distance = calculate_route_distance(reversed_route, matrix_distances)

                if distance < best_distance:
                    route = reversed_route
                    best_distance = distance

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0

    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
