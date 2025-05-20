import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize a random route
    num_cities = len(_distances)
    route = np.random.permutation(num_cities)

    # Perform local search using a 2-opt heuristic
    for _ in range(100):
        i, j = np.random.randint(num_cities, size=2)
        route = swap_edges(route, i, j)

    return route

# Helper function for swapping two edges in a route
def swap_edges(route: np.ndarray, i: int, j: int) -> np.ndarray:
    route[i], route[j] = route[j], route[i]
    return route
