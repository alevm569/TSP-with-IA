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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor
    - 2-opt

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))

    # Nearest neighbor heuristic
    current_city = start_city
    route = [current_city]

    while len(route) < len(distances):
        # Find the city with the shortest distance from the current city
        nearest_city = np.argmin(distances[current_city])
        while nearest_city in route:
            nearest_city = np.argmin(distances[current_city][np.isin(np.arange(len(distances)), route, invert=True)])

        route.append(nearest_city)
        current_city = nearest_city

    # 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_without_swap = distances[route[i]][route[(j - 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]
            distance_with_swap = distances[route[i]][route[j]] + distances[route[(j - 1) % len(route)]][route[(i + 1) % len(route)]]

            if distance_with_swap < distance_without_swap:
                route[i:j] = route[j:i:-1]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
