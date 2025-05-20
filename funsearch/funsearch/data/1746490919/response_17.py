import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
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
        - Nearest neighbor
        - Cheapest insertion

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize starting city
    current_city = 0
    route = [current_city]

    # Create a mask to track visited cities
    visited = np.zeros(len(distances), dtype=bool)
    visited[current_city] = True

    # Iterate until all cities are visited
    while not np.all(visited):
        # Get the nearest unvisited city
        nearest_city = np.argmin(distances[current_city][~visited])
        route.append(nearest_city)
        visited[nearest_city] = True
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
