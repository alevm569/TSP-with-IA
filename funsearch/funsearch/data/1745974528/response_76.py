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

    Heuristics used:
        - Nearest neighbor: Start from the first city and iteratively find the city with the shortest distance to the current city.
        - 2-opt: Swap two consecutive subroutes of the current route to potentially improve the distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route
    num_cities = len(distances)
    route = np.zeros(num_cities, dtype=int)
    route[0] = 0

    # Nearest neighbor heuristic
    current_city = 0
    for i in range(1, num_cities):
        min_distance = np.inf
        next_city = -1
        for j in range(num_cities):
            if j not in route and distances[current_city][j] < min_distance:
                min_distance = distances[current_city][j]
                next_city = j
        route[i] = next_city
        current_city = next_city

    # 2-opt heuristic
    for k in range(num_cities):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                if calculate_route_distance(route, distances) > calculate_route_distance(np.delete(route, i), distances):
                    route = np.delete(route, i)

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
