import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Create ACO object
    aco = funsearch.algorithms.aco.AntColonyOptimization(
        distance_matrix=_distances,
        population_size=100,
        num_iterations=100,
    )

    # Run ACO algorithm
    best_route = aco.run()

    # Convert ACO route to tuple
    return tuple(best_route)
