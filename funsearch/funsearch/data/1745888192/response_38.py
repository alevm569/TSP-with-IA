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

    - **Nearest neighbor:** Find the city with the shortest distance to the current city, and add it to the route.
    - **Cheapest insertion:** For each city not yet in the route, find the city in the route with the shortest distance, and insert the new city before it.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]

    # Set of cities already visited
    visited = {0}

    # Start with the first city
    current_city = 0

    # Iterate until all cities have been visited
    while len(visited) < len(matrix_distances):
        # Find the city with the shortest distance to the current city
        nearest_city = -1
        min_distance = math.inf

        for i in range(len(matrix_distances)):
            if i not in visited:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route
        route.append(nearest_city)
        visited.add(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)
