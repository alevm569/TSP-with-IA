import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of `find_best_route_v2`.

    Uses a hybrid approach combining the nearest neighbor and 2-opt heuristics.
    """

    # Initial solution using nearest neighbor
    route = nearest_neighbor(distances)

    # Local search using 2-opt to improve the solution
    route = local_search(route, distances)

    return route

# Helper functions from previous version

def nearest_neighbor(distances: np.ndarray) -> tuple[int, ...]:
    # ...

def local_search(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ...

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    # ...
