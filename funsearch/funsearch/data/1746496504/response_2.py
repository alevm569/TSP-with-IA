import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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

    Uses a combination of nearest neighbor and local search heuristics.
    """

    # Generate an initial route using nearest neighbor
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(distances))) - {current_city}

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search to refine the route
    for i in range(100):
        for j in range(len(route)):
            for k in range(j + 2, len(route)):
                new_route = route[:]
                new_route[j], new_route[k] = new_route[k], new_route[j]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += distances[route[i]][route[(i + 1) % len(route)]]
    return distance
