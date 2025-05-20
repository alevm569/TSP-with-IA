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

    Heuristics used:
    - Nearest neighbor
    - Local search with 2-opt neighborhood

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initial route using nearest neighbor
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Local search with 2-opt neighborhood
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_before = matrix_distances[route[i]][route[i+1]] + matrix_distances[route[j]][route[j-1]]
                distance_after = matrix_distances[route[i]][route[j]] + matrix_distances[route[i+1]][route[j-1]]

                if distance_after < distance_before:
                    route[i+1:j] = route[j-1:i:-1]

    # Return the route with the starting city appended
    return tuple(route + [route[0]])

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return distance
