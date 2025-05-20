import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    - Hybrid heuristic combining nearest neighbor and cheapest insertion

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the current route
    current_route = np.random.permutation(np.arange(len(distances)))

    # Perform hybrid heuristic
    while True:
        # Find the nearest unvisited city
        nearest_city = np.argmin(distances[current_route[-1]])
        if nearest_city not in current_route:
            current_route = np.append(current_route, nearest_city)

        # Find the cheapest insertion city
        cheapest_city = np.argmin(distances[current_route[:-1], current_route[1:]])
        if cheapest_city not in current_route:
            current_route = np.insert(current_route, -1, cheapest_city)

        # Check if all cities have been visited
        if len(np.unique(current_route)) == len(distances):
            break

    return current_route
