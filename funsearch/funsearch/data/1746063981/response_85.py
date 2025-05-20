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
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt operations.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform 2-opt operations to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = distances[route[i]][route[j]]
            distance_reversed = distances[route[i]][route[(j - 1)]] + distances[route[(j - 1)]][route[j]] - distance_original

            if distance_reversed < distance_original:
                route[i:j] = route[i:j][::-1]

    return tuple(route)
