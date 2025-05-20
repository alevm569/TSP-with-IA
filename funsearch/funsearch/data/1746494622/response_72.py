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

    This function uses a hybrid approach combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    # Find the remaining cities using the cheapest insertion heuristic
    remaining_cities = set(range(len(matrix_distances))) - {current_city}
    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a given route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
