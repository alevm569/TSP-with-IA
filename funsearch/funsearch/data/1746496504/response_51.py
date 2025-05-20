import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of `find_best_route_v2`. Uses a hybrid heuristic combining local search and 2-opt.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial route using nearest neighbor
    start_city = np.random.randint(len(matrix_distances))
    route = [start_city]
    unvisited = set(range(len(matrix_distances))) - {start_city}

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Local search optimization
    for _ in range(10):
        route = local_search(route, matrix_distances)

    # 2-opt optimization
    for _ in range(10):
        route = two_opt(route, matrix_distances)

    return route

def local_search(route: list[int], matrix_distances: np.ndarray) -> list[int]:
    """Performs local search optimization on a given route."""
    best_route = route[:]

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]

            # Calculate the total distance of the new route
            distance = calculate_route_distance(route, matrix_distances)

            # If the new route is better, keep it
            if distance < calculate_route_distance(best_route, matrix_distances):
                best_route = route[:]

            # Swap the cities back
            route[i], route[j] = route[j], route[i]

    return best_route

def two_opt(route: list[int], matrix_distances: np.ndarray) -> list[int]:
    """Performs 2-opt optimization on a given route."""
    best_route = route[:]

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Reverse the subroute between i and j
            route[i:j+1] = route[j:i:-1]

            # Calculate the total distance of the new route
            distance = calculate_route_distance(route, matrix_distances)

            # If the new route is better, keep it
            if distance < calculate_route_distance(best_route, matrix_distances):
                best_route = route[:]

            # Reverse the subroute back
            route[i:j+1] = route[j:i:-1]

    return best_route
