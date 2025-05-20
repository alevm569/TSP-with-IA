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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    This function combines the nearest neighbor and 2-opt heuristics to generate a candidate route.
    """
    # Initialize the route using the nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    while len(route) < len(matrix_distances):
        next_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in route])
        route.append(next_city)
        current_city = next_city

    # Apply 2-opt local search to refine the route
    for _ in range(100):
        i, j = np.random.randint(0, len(route), 2)
        route = funsearch.swap_mutation(route, i, j)

    return tuple(route)
