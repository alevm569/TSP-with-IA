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
    Improved version of find_best_route_v0.

    Uses a hybrid heuristic combining nearest neighbor and cheapest insertion.
    """
    n = len(distances)
    route = np.zeros(n + 1, dtype=int)

    # Nearest neighbor heuristic
    start_city = np.random.randint(n)
    route[0] = start_city
    unvisited = set(range(n))
    unvisited.remove(start_city)

    for i in range(1, n):
        current_city = route[i - 1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route[i] = nearest_city
        unvisited.remove(nearest_city)

    # Cheapest insertion heuristic
    for i in range(n, 0, -1):
        best_distance = np.inf
        best_city = None

        for city in unvisited:
            distance = distances[route[i]][city]
            if distance < best_distance:
                best_distance = distance
                best_city = city

        route[i] = best_city
        unvisited.remove(best_city)

    # Return to the starting city
    route[n] = start_city

    return tuple(route.astype(int))
