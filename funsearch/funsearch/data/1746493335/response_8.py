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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v0` using a hybrid approach.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid approach combining nearest neighbor, 2-opt, and local search heuristics.
    """

    # Initialize route using nearest neighbor heuristic
    route = nearest_neighbor(distances)

    # Apply 2-opt heuristic to improve the route
    route = two_opt(route, distances)

    # Perform local search to further refine the route
    route = local_search(route, distances)

    return route

# Additional helper functions can be added here, such as:

def nearest_neighbor(distances: np.ndarray) -> tuple[int, ...]:
    # Implementation of nearest neighbor heuristic

def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # Implementation of 2-opt heuristic

def local_search(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # Implementation of local search heuristic
