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
    Improved version of find_best_route_v2.
    Uses a hybrid approach combining nearest neighbor and k-opt heuristics.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(matrix_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda c: matrix_distances[current_city][c])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Apply k-opt to improve the route
    k = 2  # Number of cities to swap at each iteration
    iterations = 100  # Number of iterations to perform k-opt

    for _ in range(iterations):
        for i in range(len(route)):
            for j in range(i + k, len(route)):
                route = k_opt(route, i, j)

    return tuple(route)

# Helper function for k-opt heuristic
def k_opt(route: list[int], i: int, j: int) -> list[int]:
    # Swap the sublist of cities between indices i and j
    return route[:i] + route[j:i:-1] + route[j+1:]
