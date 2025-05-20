import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid approach combining genetic algorithms and local search.

    # Create a genetic algorithm to generate candidate routes.
    population = funsearch.genetic.population(len(_distances))

    # Use a local search algorithm to refine the candidate routes.
    population = funsearch.local_search.population(population)

    # Run the genetic algorithm for a specified number of generations.
    for generation in range(100):
        population = funsearch.genetic.population(population)
        population = funsearch.local_search.population(population)

    # Get the best route from the final population.
    best_route = population[0].route

    return best_route
