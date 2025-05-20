import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


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

    This function implements a hybrid heuristic that combines the following strategies:

    - **Nearest neighbor:** Start at an arbitrary city and iteratively select the city with the shortest distance to the current city.
    - **Cheapest insertion:** Once all cities have been visited, insert the remaining city with the lowest total distance to the current route.
    - **Local search:** Randomly swap two cities in the route and keep the best configuration.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialization
    num_cities = len(distances)
    route = np.zeros(num_cities, dtype=int)
    visited = np.zeros(num_cities, dtype=bool)

    # Nearest neighbor
    start_city = 0
    route[0] = start_city
    visited[start_city] = True
    for i in range(1, num_cities):
        next_city = np.argmin(distances[route[i - 1]][~visited])
        route[i] = next_city
        visited[next_city] = True

    # Cheapest insertion
    for i in range(num_cities, 2 * num_cities):
        city_to_add = np.argmin(np.sum(distances[route[:i - num_cities]], axis=0))
        route[i - num_cities] = city_to_add

    # Local search
    np.random.seed(42)  # For reproducibility
    for _ in range(100):
        i, j = np.random.randint(0, num_cities, size=2)
        route[i], route[j] = route[j], route[i]
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            continue
        route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
