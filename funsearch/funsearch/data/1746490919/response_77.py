import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(0)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristics."""

    # Initialize population of routes
    population = funsearch.init_population(_distances, size=100)

    # Define fitness function
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, generations=100)

    return best_route
