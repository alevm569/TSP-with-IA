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
        - Nearest neighbor: Select the city with the shortest distance to the current city.
        - Cheapest insertion: Find the city that can be added to the route with the lowest total distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]

    # Start from the first city and find the nearest neighbor
    current_city = 0
    while len(route) < len(distances):
        nearest_city = -1
        min_distance = np.inf
        for i in range(len(distances)):
            if i not in route:
                distance = distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route
        route.append(nearest_city)
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
