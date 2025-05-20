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
        - Nearest neighbor: Start from a random city and iteratively choose the nearest unvisited city.
        - Cheapest insertion: For each city in the route, find the city that minimizes the total distance when added.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with a random starting city
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    # Visit all other cities
    visited = set([current_city])
    while len(visited) < len(matrix_distances):
        # Find the nearest unvisited city
        nearest_city = np.argmin(matrix_distances[current_city][[c for c in range(len(matrix_distances)) if c not in visited]])

        # Add the nearest city to the route
        route.append(nearest_city)
        visited.add(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
