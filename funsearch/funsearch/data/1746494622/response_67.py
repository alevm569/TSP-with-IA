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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function implements a hybrid heuristic that combines the following strategies:

    - Nearest neighbor: To generate an initial feasible route.
    - Cheapest insertion: To refine the route by adding cities that minimize the total distance.
    - Local search: To explore nearby routes and find a better solution.

    Returns:
    A tuple representing the best route, including all cities and the return to the starting city.
    """

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(matrix_distances):
        current_city = route[-1]
        nearest_city = np.argmin(matrix_distances[current_city][[i for i in range(len(matrix_distances)) if i not in visited]])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Refine the route using the cheapest insertion heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_to_insert = matrix_distances[route[i]][route[j]]
            new_distance = calculate_route_distance(route[:i] + route[j:i:-1] + route[j + 1:], matrix_distances)
            if new_distance < distance_to_insert:
                route = route[:i] + route[j:i:-1] + route[j + 1:]

    # Perform local search to find a better solution
    best_route = route
    best_distance = calculate_route_distance(best_route, matrix_distances)

    for i in range(100):
        current_route = perturb(best_route)
        current_distance = calculate_route_distance(current_route, matrix_distances)

        if current_distance < best_distance:
            best_route = current_route
            best_distance = current_distance

    return best_route

def perturb(route: tuple[int, ...]) -> tuple[int, ...]:
    """Randomly permutes two cities in the route."""
    i, j = np.random.randint(0, len(route), size=2)
    route = list(route)
    route[i], route[j] = route[j], route[i]
    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
