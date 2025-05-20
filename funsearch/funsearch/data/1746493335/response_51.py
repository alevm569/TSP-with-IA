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
        - Cheapest insertion
        - Local search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Nearest neighbor heuristic
    current_city = np.random.randint(len(distances))
    route = [current_city]
    remaining_cities = set(range(len(distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Cheapest insertion heuristic
    for i in range(len(route)):
        best_distance = math.inf
        best_city = None
        for j in range(len(route)):
            if i == j:
                continue
            distance = distances[route[i]][route[j]]
            if distance < best_distance:
                best_distance = distance
                best_city = route[j]
        route.insert(i + 1, best_city)

    # Local search
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                route = new_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
