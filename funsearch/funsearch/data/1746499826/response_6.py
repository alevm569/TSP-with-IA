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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize the route with the nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    visited = set([start_city])

    # Add the remaining cities using the cheapest insertion heuristic
    for _ in range(len(_distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city])
        while nearest_city in visited:
            nearest_city = np.argmin(_distances[current_city][np.isin(_distances[current_city], _distances[current_city][visited])])
        route.append(nearest_city)
        visited.add(nearest_city)

    return tuple(route)
