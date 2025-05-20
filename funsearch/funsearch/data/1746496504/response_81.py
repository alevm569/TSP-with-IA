import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(0)  # Set a seed for reproducibility

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

    This function implements a hybrid approach combining nearest neighbor, 2-opt, and local search heuristics.
    """

    # Initialize a random route
    n = distances.shape[0]
    route = np.random.permutation(n)

    # Nearest neighbor heuristic
    if np.random.rand() < 0.3:
        current_city = route[0]
        remaining_cities = set(range(n)) - {current_city}
        for _ in range(n - 1):
            closest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
            route[np.where(route == current_city)[0][-1] + 1] = closest_city
            remaining_cities.remove(closest_city)
            current_city = closest_city

    # 2-opt heuristic
    if np.random.rand() < 0.5:
        for _ in range(100):
            i, j = np.random.randint(0, n, 2)
            route[i:j+1] = route[i:j+1][::-1]

    # Local search heuristic
    for _ in range(100):
        for i in range(n):
            for j in range(i+1, n):
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
