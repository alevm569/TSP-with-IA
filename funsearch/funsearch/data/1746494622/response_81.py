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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2, using a combination of local search and 2-opt heuristics.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial solution using the nearest neighbor heuristic
    initial_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Perform local search to find a better solution
    best_route = funsearch.local_search(initial_route, matrix_distances)

    # Apply 2-opt heuristics to further improve the solution
    best_route = funsearch.two_opt(best_route, matrix_distances)

    return best_route
