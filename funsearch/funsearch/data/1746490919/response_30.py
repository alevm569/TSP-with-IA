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

    # Initialize population of random routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Perform genetic algorithm optimization
    best_route = funsearch.genetic_algorithm(population, _distances, max_generations=100)

    return best_route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with local search."""

    # Use ACO algorithm to generate initial population
    population = funsearch.aco(_distances, num_ants=100)

    # Perform local search optimization on the best route found by ACO
    best_route = funsearch.local_search(population[0], _distances)

    return best_route
