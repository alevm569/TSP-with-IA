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

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Strategies used:
        - Hybrid heuristic: Combines two heuristics:
            - Nearest neighbor: Starts from an initial city and iteratively selects the nearest unvisited city.
            - Cheapest insertion: Selects the city that minimizes the total distance when added to the current route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]
    visited = set([0])

    # Nearest neighbor heuristic
    for _ in range(len(_distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city][~np.isin(np.arange(len(_distances)), visited)])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Cheapest insertion heuristic
    while len(visited) < len(_distances):
        current_city = route[-1]
        cheapest_city = np.argmin(_distances[current_city][~np.isin(np.arange(len(_distances)), visited)])
        route.append(cheapest_city)
        visited.add(cheapest_city)

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
