import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use genetic algorithms with a modified fitness function.
    def fitness(route):
        total_distance = calculate_route_distance(route, _distances)
        # Penalize invalid routes (e.g., repeated or missing cities).
        penalty = 0
        for i in range(len(route)):
            if route[i] in route[:i] or route[i] not in range(len(_distances)):
                penalty += math.inf
        return total_distance + penalty

    # Initialize population.
    population = np.random.permutation(np.arange(len(_distances)))

    # Run genetic algorithm.
    best_route = funsearch.genetic_algorithm(fitness, population, max_generations=100)

    return best_route
