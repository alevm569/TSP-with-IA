import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of find_best_route_v2, using a hybrid heuristic.

    Hybrid Heuristic:
        - Start with a random permutation of cities.
        - Use the 2-opt heuristic to iteratively swap two cities in the route to improve the distance.
        - Use the local search heuristic to further refine the route by iteratively making small changes.

    Parameters:
        matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Generate a random initial route
    num_cities = len(matrix_distances)
    initial_route = np.random.permutation(num_cities)

    # Use the 2-opt heuristic to improve the route
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            initial_route = two_opt(initial_route, i, j)

    # Use the local search heuristic to further refine the route
    best_route = local_search(initial_route, matrix_distances)

    return best_route

def two_opt(route: np.ndarray, i: int, j: int) -> np.ndarray:
    """
    Performs the 2-opt swap operation on a route.
    """
    return np.append(route[:i], np.append(route[j:i:-1], route[j+1:]))

def local_search(route: np.ndarray, matrix_distances: np.ndarray) -> np.ndarray:
    """
    Performs local search on a route by iteratively swapping two cities.
    """
    best_distance = calculate_route_distance(route, matrix_distances)

    while True:
        improved = False

        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = two_opt(route, i, j)
                new_distance = calculate_route_distance(new_route, matrix_distances)

                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance
                    improved = True

        if not improved:
            break

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0

    for i in range(len(route)):
        j = (i + 1) % len(route)
        total_distance += matrix_distances[route[i]][route[j]]

    return total_distance
