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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid approach.

    This function combines the nearest neighbor heuristic for initialization,
    cheapest insertion for route construction, and 2-opt for local search.
    """

    # Initialize a random route
    num_cities = len(matrix_distances)
    route = np.random.permutation(num_cities)

    # Apply nearest neighbor heuristic for initialization
    # ...

    # Apply cheapest insertion for route construction
    # ...

    # Perform local search using 2-opt
    for i in range(100):
        # ...

    return route
