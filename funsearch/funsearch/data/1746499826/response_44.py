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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Use a hybrid heuristic that combines multiple strategies:

    - Nearest neighbor to generate an initial route.
    - Cheapest insertion to refine the route.
    - Tabu search with 2-opt and 3-opt neighborhood operators.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Refine the route using the cheapest insertion heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if distances[route[i]][route[j]] > distances[route[i]][route[j - 1]]:
                route[i+1:j] = route[j-1:i:-1]

    # Perform tabu search with 2-opt and 3-opt neighborhood operators
    def tabu_search(route):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                for k in range(j + 1, len(route)):
                    if (
                        distances[route[i]][route[j]] + distances[route[j]][route[k]] + distances[route[k]][route[i]]
                        < distances[route[i]][route[k]] + distances[route[k]][route[j]] + distances[route[j]][route[i]]
                    ):
                        route[i+1:j] = route[j:i:-1]

        return route

    for i in range(100):
        route = tabu_search(route)

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """
    distance = 0
    for i in range(len(route)):
        distance += distances[route[i]][route[(i + 1) % len(route)]]

    return distance
