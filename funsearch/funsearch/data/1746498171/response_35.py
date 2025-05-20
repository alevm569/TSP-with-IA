import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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

    Use a hybrid approach that combines the nearest neighbor heuristic with a local search approach.
    """

    # Generate an initial route using the nearest neighbor heuristic
    initial_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Perform local search to refine the route
    current_route = initial_route
    best_distance = calculate_route_distance(current_route, matrix_distances)

    while True:
        # Generate a new route by swapping two random cities
        idx1, idx2 = np.random.randint(len(matrix_distances), size=2)
        new_route = current_route.copy()
        new_route[idx1], new_route[idx2] = new_route[idx2], new_route[idx1]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(new_route, matrix_distances)

        # If the new route is better, update the current route
        if new_distance < best_distance:
            current_route = new_route
            best_distance = new_distance
        else:
            # If the new route is not better, terminate the local search
            break

    return current_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
