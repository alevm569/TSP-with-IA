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

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use ACO with a heuristic solution to get a good starting point
    best_route = funsearch.aco.aco(
        funsearch.heuristic.nearest_neighbor(_distances),
        funsearch.distance.total_distance(_distances),
        seed=42,
    )

    # Perform local search to refine the route
    best_route = funsearch.local_search.simulated_annealing(
        funsearch.distance.total_distance(_distances),
        best_route,
        seed=42,
    )

    return best_route
