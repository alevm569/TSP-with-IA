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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Initialize a random seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic combining local search and 2-opt
    def hybrid_heuristic(route):
        # Perform local search to improve the current route
        route = funsearch.localsearch.random_restart_local_search(route, _distances)

        # Apply 2-opt to further optimize the route
        route = funsearch.route.two_opt(route, _distances)

        return route

    # Use the hybrid heuristic and the ACO algorithm
    best_route = funsearch.optimize.hybrid_aco_greedy(
        hybrid_heuristic, _distances, seed=42, n_iterations=1000
    )

    return best_route
