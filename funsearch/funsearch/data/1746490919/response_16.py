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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Initialize route using nearest neighbor heuristic
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Use 2-opt heuristic to improve the route
    for _ in range(len(_distances) - 1):
        best_distance = float('inf')
        for i in range(len(_distances)):
            if i in route:
                continue
            new_distance = calculate_route_distance(route + [i], _distances)
            if new_distance < best_distance:
                best_distance = new_distance
                best_city = i
        route.append(best_city)

    # Return to starting city
    route.append(route[0])

    return tuple(route)
