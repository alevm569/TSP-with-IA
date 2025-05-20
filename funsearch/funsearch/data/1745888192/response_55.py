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
    Improved version of `find_best_route_v2`.

    Uses a combination of local search and 2-opt heuristics.
    """

    # Initialize a random route
    route = np.random.permutation(np.arange(len(matrix_distances)))

    # Perform local search
    for i in range(100):
        # Choose two random cities
        idx1, idx2 = np.random.randint(0, len(route), size=2)

        # Swap the two cities in the route
        route[idx1], route[idx2] = route[idx2], route[idx1]

        # Check if the new route is better
        if calculate_route_distance(route, matrix_distances) < calculate_route_distance(route, matrix_distances):
            pass  # Keep the new route
        else:
            route[idx1], route[idx2] = route[idx2], route[idx1]  # Swap them back

    # Perform 2-opt optimization
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j+1:]
            if calculate_route_distance(new_route, matrix_distances) < calculate_route_distance(route, matrix_distances):
                route = new_route

    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
