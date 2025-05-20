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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    This function combines the nearest neighbor heuristic for initialization
    and the 2-opt heuristic for local search.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialization using nearest neighbor heuristic
    num_cities = len(distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    while len(route) < num_cities:
        closest_city = np.argmin(distances[current_city])
        if closest_city not in route:
            route.append(closest_city)
            current_city = closest_city

    # Local search using 2-opt heuristic
    for i in range(100):  # Number of iterations
        for j in range(len(route)):
            for k in range(j + 2, len(route)):
                new_route = route[:j] + route[k:j:-1] + route[k + 1:]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
