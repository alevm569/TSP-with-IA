import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic combining nearest neighbor and ACO
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor(_distances),
        funsearch.aco(_distances)
    )

    # Perform local search to refine the route
    best_route = funsearch.local_search(best_route, _distances)

    return best_route
