import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic combining nearest neighbor and 2-opt
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor,
        funsearch.two_opt,
        distance_matrix=_distances,
        iterations=1000,
        population_size=100,
        verbose=False,
    )

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3`."""
    # Use a genetic algorithm with a custom fitness function
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    best_route = funsearch.genetic(
        fitness,
        population_size=100,
        generations=1000,
        verbose=False,
    )

    return best_route
