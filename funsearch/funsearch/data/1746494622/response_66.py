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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Combines nearest neighbor and 2-opt heuristics to find an initial feasible solution.
    Then, performs local search with 2-opt moves to refine the route.

    Args:
        distances: A square matrix of distances between cities.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial solution using nearest neighbor
    initial_route = funsearch.nearest_neighbor(distances)

    # Refine the solution using local search with 2-opt moves
    best_route = funsearch.local_search(distances, initial_route, funsearch.two_opt)

    return best_route
