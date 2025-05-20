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

    Heuristics used:
    - nearest neighbor
    - local search with k-opt (k = 2)

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    for _ in range(len(distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin(distances[current_city][~np.isin(np.arange(len(distances)), visited)])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Perform local search using k-opt (k = 2)
    for _ in range(100):  # Number of iterations
        funsearch.kopt(distances, route)

    return tuple(route)
