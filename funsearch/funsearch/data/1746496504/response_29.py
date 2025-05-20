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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using ACO.
    """
    num_cities = len(distances)

    # Create an ACO optimizer with the given distances
    optimizer = funsearch.ACOOptimizer(distances)

    # Run the optimizer for a specified number of iterations
    optimizer.optimize(num_iterations=100)

    # Return the best route found by the ACO algorithm
    return optimizer.best_route
