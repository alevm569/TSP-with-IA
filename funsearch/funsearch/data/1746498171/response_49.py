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
    - 2-opt

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting route
    np.random.seed(42)  # Set seed for reproducibility
    route = np.random.permutation(distances.shape[0])

    # Perform 2-opt neighborhood search
    for _ in range(1000):  # Number of iterations
        # Randomly select two cities in the route
        i, j = np.random.randint(0, distances.shape[0], size=2)

        # Calculate the distance difference if we swap these two cities
        distance_diff = distances[route[i-1]][route[i]] + distances[route[j-1]][route[j]] - distances[route[i-1]][route[j]] - distances[route[j-1]][route[i]]

        # If swapping these two cities improves the route distance, do it
        if distance_diff < 0:
            route[i:j+1] = route[j:i-1:-1]

    return route
