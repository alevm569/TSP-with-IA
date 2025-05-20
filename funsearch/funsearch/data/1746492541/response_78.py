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
    Improved version of `find_best_route_v2` using a hybrid approach combining 2-opt and local search.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt local search to optimize the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                if matrix_distances[route[i]][route[j]] < matrix_distances[route[i]][route[(j - 1)]] + matrix_distances[route[j]][route[(i + 1)]]:
                    route[i:j] = route[j - 1:i:-1]

    return tuple(route)
