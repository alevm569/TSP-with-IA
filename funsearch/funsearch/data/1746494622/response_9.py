import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristics."""

    # Hybrid heuristic using nearest neighbor and 2-opt
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor(_distances),
        funsearch.two_opt(_distances),
    )

    return best_route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with ACO."""

    # Ant colony optimization with best route as pheromone trail
    best_route = funsearch.aco(
        _distances,
        pheromone_trail=funsearch.best_route_pheromone_trail,
    )

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` with genetic algorithms."""

    # Genetic algorithm with crossover and mutation operators
    best_route = funsearch.genetic(
        _distances,
        crossover=funsearch.uniform_crossover,
        mutation=funsearch.swap_mutation,
    )

    return best_route
