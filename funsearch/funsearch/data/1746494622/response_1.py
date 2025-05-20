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
    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic that combines nearest neighbor and 2-opt
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor,
        funsearch.two_opt,
        _distances,
        max_iterations=1000
    )

    return best_route


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Further improved version of `find_best_route_v1`."""
    # Use a genetic algorithm to search for the optimal route
    population_size = 100
    num_generations = 100
    tournament_size = 5

    best_route = funsearch.genetic_algorithm(
        population_size,
        num_generations,
        tournament_size,
        _distances
    )

    return best_route
