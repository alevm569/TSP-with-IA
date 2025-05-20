import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Apply 2-opt heuristic to improve initial solution
    best_route = find_best_route_v0(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        # Randomly choose two cities in the route
        city1, city2 = np.random.choice(len(_distances), size=2, replace=False)

        # Swap the two cities in the route
        new_route = best_route[:city1] + best_route[city2:city1:-1] + best_route[city2+1:]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(new_route, _distances)

        # If the new route is shorter, accept it
        if new_distance < best_distance:
            best_route = new_route
            best_distance = new_distance

        # Otherwise, reject the new route and continue iterating
        else:
            continue

    return best_route
