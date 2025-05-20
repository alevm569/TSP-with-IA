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
    """Improved version of `find_best_route_v2` with new heuristics."""

    # Initialize random seed for reproducibility
    np.random.seed(0)

    # Use a hybrid heuristic that combines two or more of the following:
    # - Local search with 2-opt moves
    # - Ant colony optimization (ACO)
    # - Genetic algorithms
    # - Tabu search

    # Example using ACO:
    num_ants = 10
    iterations = 100

    # Create ACO algorithm
    aco = funsearch.algorithms.ACO(_distances, num_ants, iterations)

    # Run ACO algorithm
    best_route = aco.run()

    # Return the best route found
    return best_route
