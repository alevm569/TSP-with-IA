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

    Using a combination of nearest neighbor and 2-opt heuristics.
    """

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        current_city = route[-1]
        closest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(closest_city)
        remaining_cities.remove(closest_city)

    # Improve the route using 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = matrix_distances[route[i]][route[j]]

            # Swap the two cities
            route[i], route[j] = route[j], route[i]

            # Calculate the distance of the swapped route
            distance_swapped = matrix_distances[route[i]][route[j]]

            # If the swapped route is shorter, keep the swap
            if distance_swapped < distance_original:
                pass
            else:
                # If the swapped route is longer, reverse the swap
                route[i], route[j] = route[j], route[i]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
