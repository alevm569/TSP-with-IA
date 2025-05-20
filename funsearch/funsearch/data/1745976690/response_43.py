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

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """

    # Implement your new version of the find_best_route function here.
    # You can use any of the provided heuristics or create your own.

    # Example heuristic:
    # Start from the first city and iteratively add the nearest unvisited city.

    # Initialize starting city
    current_city = 0
    route = [current_city]

    # Visit all cities except the starting city
    remaining_cities = set(range(1, len(matrix_distances)))

    # Keep adding the nearest unvisited city until all cities are visited
    while remaining_cities:
        # Find the nearest unvisited city
        nearest_city = min(remaining_cities, key=lambda c: matrix_distances[current_city][c])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the set of remaining cities
        remaining_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)
