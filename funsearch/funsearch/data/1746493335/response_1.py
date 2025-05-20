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

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the 2-opt heuristic to improve the route
    best_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        # Randomly select two cities in the route
        i, j = np.random.randint(0, len(_distances), size=2)

        # Create a new route by swapping the two cities
        new_route = best_route[:i] + best_route[i+1:j+1] + [best_route[i]] + best_route[j+1:]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(new_route, _distances)

        # If the new route is shorter, update the best route
        if new_distance < best_distance:
            best_route = new_route
            best_distance = new_distance
        else:
            break

    return best_route
