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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Implement a hybrid heuristic that combines two or more of the following:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - ACO (ant colony optimization)
    # - genetic algorithms
    # - k-opt
    # - tabu search

    # For example, you could combine the nearest neighbor heuristic with the ACO heuristic:

    # 1. Initialize a random route.
    route = np.random.permutation(np.arange(_distances.shape[0]))

    # 2. Use the ACO heuristic to improve the route.
    aco = funsearch.aco.AntColonyOptimizer()
    aco.optimize(route, _distances)

    # 3. Return the best route found.
    return route
