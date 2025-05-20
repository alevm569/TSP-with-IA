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
    Improved version of find_best_route_v1.

    Uses a hybrid heuristic that combines:
        - Nearest neighbor for initial route construction.
        - Cheapest insertion to fill in missing cities.
        - Tabu search to refine the route.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initial route using nearest neighbor
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(distances):
        closest_city = np.argmin(distances[current_city])
        if closest_city not in route:
            route.append(closest_city)
            current_city = closest_city

    # Fill in missing cities using cheapest insertion
    remaining_cities = set(range(len(distances))) - set(route)
    while remaining_cities:
        cheapest_city = min(remaining_cities, key=lambda city: distances[route[-1]][city])
        route.append(cheapest_city)
        remaining_cities.remove(cheapest_city)

    # Refine route using tabu search
    tabu_size = 10
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                if calculate_route_distance(new_route, distances) < calculate_route_distance(route, distances):
                    route = new_route

    return tuple(route)
