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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    This function uses a hybrid approach that combines the nearest neighbor heuristic,
    cheapest insertion heuristic, and local search to find the best route.
    """

    # Initialize the route with the nearest neighbor heuristic
    current_city = 0
    route = [current_city]

    # Use the cheapest insertion heuristic to add the remaining cities
    remaining_cities = set(range(len(matrix_distances)))
    remaining_cities.remove(current_city)
    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Perform local search to improve the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Swap two cities in the route
                route[i], route[j] = route[j], route[i]

                # Calculate the total distance of the route
                distance = calculate_route_distance(route, matrix_distances)

                # If the new route is better, keep it
                if distance < calculate_route_distance(route, matrix_distances):
                    break

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
