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

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Perform nearest neighbor to get an initial solution
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        closest_city = np.argmin(_distances[current_city])
        if closest_city not in route:
            route.append(closest_city)
            current_city = closest_city

    # Use 2-opt local search to improve the solution
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j + 1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
