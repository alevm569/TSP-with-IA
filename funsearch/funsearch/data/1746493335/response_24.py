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

    # Implement a new version of the find_best_route function here.
    # Use the following heuristics and strategies:
    # - Local search with multiple restarts
    # - 2-opt with a probability of exploration
    # - ACO with a pheromone evaporation rate

    # Example implementation using ACO with local search:
    best_route = funsearch.aco.aco_with_local_search(_distances, max_iter=1000, population_size=100, elitism_rate=0.1, pheromone_evaporation_rate=0.9)

    return best_route
