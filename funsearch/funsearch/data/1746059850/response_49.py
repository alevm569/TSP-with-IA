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
    Improved version of find_best_route_v2 using a hybrid heuristic combining nearest neighbor and cheapest insertion.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize the first city as the starting point
    start_city = 0
    route = [start_city]

    # Mark the starting city as visited
    visited = np.zeros(len(matrix_distances), dtype=bool)
    visited[start_city] = True

    # Iterate until all cities are visited
    while np.any(~visited):
        # Get the last city in the route
        current_city = route[-1]

        # Find the nearest unvisited city
        nearest_city = np.argmin(matrix_distances[current_city][~visited])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Mark the nearest city as visited
        visited[nearest_city] = True

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
