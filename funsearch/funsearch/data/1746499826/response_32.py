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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic combining the nearest neighbor and 2-opt techniques.
    """

    # Initialize a random route
    current_route = np.random.permutation(len(distances))

    # Use nearest neighbor to fill in gaps in the route
    for i in range(len(distances)):
        if i not in current_route:
            current_route = np.insert(current_route, i, find_nearest_city(current_route, i, distances))

    # Use 2-opt to refine the route
    for _ in range(100):  # Adjust the number of iterations as needed
        i, j = np.random.randint(0, len(distances), 2)
        current_route = two_opt(current_route, i, j)

    return current_route
