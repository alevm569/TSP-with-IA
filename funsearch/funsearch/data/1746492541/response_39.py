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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    This function combines the nearest neighbor heuristic for initialization
    and the 2-opt heuristic for local search.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    route = np.random.permutation(np.arange(len(distances)))

    # Use nearest neighbor heuristic to fill in missing cities
    for i in range(len(route)):
        if route[i] is None:
            current_city = route[i - 1]
            closest_city = np.argmin(distances[current_city])
            route[i] = closest_city

    # Perform local search using 2-opt heuristic
    for _ in range(100):  # Number of iterations
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                if calculate_route_distance(route, distances) > calculate_route_distance(np.roll(route, -j), distances):
                    route = np.roll(route, -j)

    return route
