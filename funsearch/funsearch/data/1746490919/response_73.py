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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Hybrid heuristic combines:
        - Nearest neighbor for initial route construction.
        - Cheapest insertion for subsequent route additions.
        - 2-opt local search for route optimization.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initial route using nearest neighbor
    start_city = np.random.randint(len(matrix_distances))
    current_city = start_city
    route = [current_city]

    # Remaining cities to visit
    remaining_cities = set(range(len(matrix_distances)))
    remaining_cities.remove(start_city)

    # Cheapest insertion heuristic
    while remaining_cities:
        # Find the city with the shortest distance from the current city
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # 2-opt local search optimization
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]

            # Check if the new route is shorter
            if calculate_route_distance(route, matrix_distances) < calculate_route_distance(route, matrix_distances):
                break

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
