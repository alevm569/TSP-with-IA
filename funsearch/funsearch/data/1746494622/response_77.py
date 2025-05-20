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


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid approach combining genetic algorithms and local search.

    # Generate candidate routes using a genetic algorithm.
    population = funsearch.genetic_algorithm(_distances)

    # Refine the candidate routes using a local search algorithm.
    population = funsearch.local_search(population, _distances)

    # Select the best route from the population.
    best_route = funsearch.best_route(population, _distances)

    return best_route
