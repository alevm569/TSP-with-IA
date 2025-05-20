import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with stability and new heuristics."""

    # Set random seed for reproducibility
    np.random.seed(42)

    # Initialize the search space with the set of all city indices
    search_space = list(range(len(_distances)))

    # Use a hybrid heuristic combining local search and 2-opt
    best_route = funsearch.hybrid_heuristic(
        search_space,
        funsearch.local_search,
        funsearch.two_opt,
        distance_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    return best_route


def calculate_route_distance(route: tuple[int], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
