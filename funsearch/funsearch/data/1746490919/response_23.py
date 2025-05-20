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
    Improved version of find_best_route_v2.

    This function uses a hybrid heuristic combining local search with 2-opt.
    """

    # Initialize a random route
    np.random.seed(42)  # Set a seed for reproducibility
    route = np.random.permutation(len(distances))

    # Perform local search
    for _ in range(100):  # Run for 100 iterations
        # Find two random cities to swap
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            pass  # Keep the new route
        else:
            route[i], route[j] = route[j], route[i]  # Swap the cities back

    # Perform 2-opt optimization
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route.copy()
            new_route[i:j+1] = new_route[j:i:-1]  # Reverse the subsequence
            if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                route = new_route

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
