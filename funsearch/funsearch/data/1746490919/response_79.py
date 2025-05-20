import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    # Use a hybrid approach combining ACO and k-opt
    aco = funsearch.ACO(distance_matrix=_distances)
    kopt = funsearch.KOPT(distance_matrix=_distances)

    # Run the ACO algorithm for 100 iterations
    best_route_aco = aco.run(max_iterations=100)

    # Run the k-opt algorithm on the best route found by ACO
    best_route_kopt = kopt.run(route=best_route_aco)

    return best_route_kopt
