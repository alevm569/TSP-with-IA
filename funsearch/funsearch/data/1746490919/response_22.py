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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Uses a hybrid heuristic that combines nearest neighbor and k-opt.
    """

    # Initialize a random route
    route = np.random.permutation(np.arange(len(distances)))

    # Use nearest neighbor to fill in any missing cities
    for i in range(len(route)):
        if route[i] is None:
            route[i] = np.argmin(distances[route[i-1]])

    # Perform k-opt local search to improve the route
    for _ in range(100):
        k = np.random.randint(2, len(route))
        route = funsearch.kopt(route, distances, k)

    return route
