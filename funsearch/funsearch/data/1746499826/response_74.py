import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize population with random routes
    population = [np.random.permutation(np.arange(len(_distances))) for _ in range(100)]

    # Define the fitness function
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, generations=1000)

    return best_route
