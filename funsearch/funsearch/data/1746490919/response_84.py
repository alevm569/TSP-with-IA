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
    Improved version of find_best_route_v2.

    Uses a combination of local search and a 2-opt heuristic.
    """

    # Generate an initial random route
    num_cities = distances.shape[0]
    initial_route = np.random.permutation(num_cities)

    # Perform local search to refine the route
    best_route = local_search(initial_route, distances)

    # Use 2-opt heuristic to further improve the route
    best_route = two_opt(best_route, distances)

    return best_route


def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Performs local search to find the best route in the neighborhood."""
    # TODO: Implement local search algorithm here.
    return route


def two_opt(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Performs the 2-opt heuristic to improve the route."""
    # TODO: Implement 2-opt heuristic here.
    return route
