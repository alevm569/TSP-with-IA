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

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Ensure reproducibility by setting a seed for random number generation.
    np.random.seed(42)

    # Use a metaheuristic algorithm, such as simulated annealing or tabu search.
    best_route = funsearch.tabu_search(
        funsearch.fitness_function(calculate_route_distance, args=(_distances,)),
        funsearch.initial_solution_generator(lambda: np.random.permutation(np.arange(_distances.shape[0]))),
        funsearch.termination_condition(max_iterations=1000),
        funsearch.tabu_tenure(decay_factor=0.9),
    )

    return best_route
