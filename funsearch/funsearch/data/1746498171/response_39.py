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
    Improved version of find_best_route_v2 using hybrid heuristics.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Hybrid heuristic using nearest neighbor and k-opt
    route = funsearch.hybrid_heuristic(distances, funsearch.nearest_neighbor, funsearch.k_opt)

    # Local search to improve the solution
    route = funsearch.local_search(distances, route)

    return route
