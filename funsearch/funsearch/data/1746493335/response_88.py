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

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Apply a combination of heuristics:
    # 1. Use nearest neighbor to initialize a route.
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: matrix_distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # 2. Apply 2-opt to improve the route quality.
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = matrix_distances[route[i]][route[j]]
            route[i+1:j] = reversed(route[i+1:j])
            distance_after = matrix_distances[route[i]][route[j]]
            if distance_after < distance_before:
                break

    # 3. Perform local search to refine the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route.copy()
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, matrix_distances) < calculate_route_distance(route, matrix_distances):
                route = new_route

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
