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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:

    - **Nearest neighbor:** Start from the first city and iteratively select the city with the shortest distance to the current city.
    - **Cheapest insertion:** For each city in the route, find the city that minimizes the total distance if added to the route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]

    # Create a mask to track visited cities
    visited = np.zeros(len(matrix_distances), dtype=bool)
    visited[0] = True

    # Nearest neighbor heuristic
    while not np.all(visited):
        current_city = route[-1]
        min_distance = float('inf')

        # Find the city with the shortest distance to the current city
        for i in range(len(matrix_distances)):
            if not visited[i] and matrix_distances[current_city][i] < min_distance:
                min_distance = matrix_distances[current_city][i]
                next_city = i

        # Add the next city to the route
        route.append(next_city)
        visited[next_city] = True

    # Return to the starting city
    route.append(0)

    return tuple(route)
