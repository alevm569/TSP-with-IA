import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use genetic algorithms with a fitness function that calculates the total distance of a route.
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Create a population of routes.
    population = funsearch.population(fitness, 100)

    # Run the genetic algorithm for 100 generations.
    best_route = funsearch.genetic_algorithm(fitness, population, generations=100)

    # Return the best route found.
    return best_route
