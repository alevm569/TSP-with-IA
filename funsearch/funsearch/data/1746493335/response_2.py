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

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Create an initial route using the nearest neighbor heuristic
    initial_route = np.random.permutation(np.arange(len(_distances)))

    # Use a local search algorithm with the 2-opt move operator to refine the route
    best_route = funsearch.greedy.local_search(initial_route, _distances, move_operator=funsearch.operators.two_opt)

    return best_route
