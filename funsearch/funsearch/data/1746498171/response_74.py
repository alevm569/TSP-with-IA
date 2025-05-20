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

    Use a combination of the following heuristics:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a list of unvisited cities
    unvisited = list(range(len(distances)))

    # Start from the first city
    current_city = 0
    route = [current_city]

    # Visit all cities once
    while len(unvisited) > 0:
        # Find the nearest unvisited city
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
