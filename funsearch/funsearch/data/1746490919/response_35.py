import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial random route
    num_cities = distances.shape[0]
    initial_route = np.random.permutation(num_cities)

    # Local search with 2-opt move
    best_route = local_search(initial_route, distances)

    return best_route

def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """
    Performs local search to improve a given route.

    Parameters:
    route (np.ndarray): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    np.ndarray: The improved route.
    """

    improved = True
    while improved:
        improved = False
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Perform 2-opt move
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]

                # Check if the new route is better
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route
                    improved = True

    return route

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (np.ndarray): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]

    return total_distance
