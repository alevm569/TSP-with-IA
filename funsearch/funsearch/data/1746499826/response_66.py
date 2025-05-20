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
    - Local search with a 2-opt neighborhood operator.
    - Randomly swap two cities in the route with a probability of 0.05.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random route.
    np.random.seed(42)
    route = np.random.permutation(len(distances))

    # Local search with 2-opt neighborhood operator.
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                # Swap two cities in the route.
                route[i], route[j] = route[j], route[i]

                # Check if the new route is better.
                if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
                    break

        # Randomly swap two cities in the route with a probability of 0.05.
        if np.random.rand() < 0.05:
            i, j = np.random.randint(0, len(route), 2)
            route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
