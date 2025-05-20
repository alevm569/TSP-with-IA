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


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Initialize ACO parameters
    num_ants = 10
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    Q = 1.0

    # Create ACO model
    model = funsearch.ACOModel(_distances, num_ants, alpha, beta, rho, Q)

    # Run ACO optimization
    model.optimize()

    # Get the best route
    best_route = model.best_route

    return best_route
