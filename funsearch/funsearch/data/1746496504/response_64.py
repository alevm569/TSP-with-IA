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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a hybrid heuristic.

    Parameters:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    num_cities = distances.shape[0]
    route = np.random.permutation(num_cities)

    # Perform local search using 2-opt heuristic
    for _ in range(100):
        i, j = np.random.randint(0, num_cities, 2)
        route = funsearch.local_search.two_opt(route, distances)

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
        route (tuple[int, ...]): A permutation of cities.
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i], route[(i + 1) % len(route)]]

    return total_distance
