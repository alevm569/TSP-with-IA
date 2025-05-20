import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(0)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid approach."""

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Use local search with the 2-opt heuristic to improve the route
    best_route = funsearch.local_search(route, _distances, funsearch.two_opt)

    # Return the best route
    return best_route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Use genetic algorithms to find the best route
    best_route = funsearch.genetic_algorithm(len(_distances), _distances)

    # Return the best route
    return best_route
