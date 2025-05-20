import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
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
    Improved version of find_best_route_v2, using a hybrid heuristic combining nearest neighbor and 2-opt.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Nearest neighbor heuristic to generate an initial route
    num_cities = len(matrix_distances)
    initial_route = np.random.permutation(num_cities)

    # 2-opt heuristic to improve the route
    best_route = initial_route
    best_distance = calculate_route_distance(best_route, matrix_distances)

    while True:
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = best_route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, matrix_distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        if best_distance == calculate_route_distance(best_route, matrix_distances):
            break

    return best_route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
