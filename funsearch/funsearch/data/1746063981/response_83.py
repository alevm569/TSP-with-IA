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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristic:
        - Use the nearest neighbor algorithm to generate an initial route.

    Local search:
        - Apply the 2-opt neighborhood operation to improve the route.

    Route must include all cities exactly once and return to the starting point.
    """

    # Generate an initial route using the nearest neighbor algorithm
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Apply the 2-opt neighborhood operation to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = matrix_distances[route[i]][route[j]]
            distance_reversed = matrix_distances[route[i]][route[(j - 1)]] + matrix_distances[route[j]][route[(i + 1)]]
            if distance_reversed < distance_original:
                route[i:j] = route[j - 1:i:-1]

    return tuple(route)
