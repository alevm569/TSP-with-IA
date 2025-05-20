import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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

    Hybrid heuristic combining nearest neighbor, cheapest insertion, and local search.
    """

    # Nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    while len(route) < len(matrix_distances):
        nearest_city = np.argmin(matrix_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Cheapest insertion heuristic
    for i in range(len(matrix_distances)):
        if i not in route:
            min_distance = np.min(matrix_distances[route[-1]])
            min_city = np.argmin(matrix_distances[route[-1]])
            if matrix_distances[route[-1]][i] < min_distance:
                route.append(i)

    # Local search heuristic
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
