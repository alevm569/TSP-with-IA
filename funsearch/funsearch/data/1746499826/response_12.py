import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set random seed for reproducibility

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

    # Hybrid heuristic combining nearest neighbor and 2-opt
    initial_route = funsearch.nearest_neighbor(_distances)
    optimized_route = funsearch.two_opt(initial_route, _distances)

    return optimized_route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with genetic algorithms."""

    # Genetic algorithm optimization with fitness function as total distance
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    best_route, _ = funsearch.genetic_algorithm(
        population_size, generations, crossover_rate, mutation_rate, _distances
    )

    return best_route
