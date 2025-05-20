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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Heuristics used:
    - Nearest neighbor: Start from the first city and iteratively select the closest unvisited city.
    - Local search: Apply the 2-opt heuristic to iteratively swap two edges and check if it improves the route distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route
    num_cities = len(matrix_distances)
    route = np.zeros(num_cities, dtype=int)
    visited = np.zeros(num_cities, dtype=bool)

    # Nearest neighbor heuristic
    current_city = 0
    for i in range(num_cities):
        visited[current_city] = True
        route[i] = current_city
        next_city = np.argmin([matrix_distances[current_city][j] for j in range(num_cities) if not visited[j]])
        current_city = next_city

    # Local search
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            new_route = route.copy()
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, matrix_distances) < calculate_route_distance(route, matrix_distances):
                route = new_route

    return route.astype(int)


def calculate_route_distance(route: np.ndarray, matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
