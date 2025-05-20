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

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Start with the nearest neighbor heuristic
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        # Find the closest unvisited city
        min_distance = math.inf
        next_city = None
        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route
        route.append(next_city)
        current_city = next_city

    # Perform local search to refine the route
    funsearch.local_search(route, _distances)

    return tuple(route)
