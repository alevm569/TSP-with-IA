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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Use a hybrid approach that combines nearest neighbor and 2-opt heuristics.
    """

    # Generate an initial route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform 2-opt optimization on the initial route
    def two_opt(route):
        best_distance = calculate_route_distance(route, distances)

        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                new_route = route[:i] + route[i:j][::-1] + route[j:]
                new_distance = calculate_route_distance(new_route, distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route

        return best_route, best_distance

    # Perform 2-opt optimization multiple times
    for _ in range(10):
        route, _ = two_opt(route)

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
