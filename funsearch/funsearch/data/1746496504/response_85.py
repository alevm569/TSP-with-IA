import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
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
    Improved version of find_best_route_v2, incorporating a hybrid heuristic.
    """
    # Use nearest neighbor to initialize the route
    current_city = 0
    route = [current_city]

    # Iterate until all cities are visited
    while len(route) < len(matrix_distances):
        # Find the nearest unvisited city
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in route])
        route.append(nearest_city)
        current_city = nearest_city

    # Add the starting city back to the route
    route.append(route[0])

    return tuple(route)
