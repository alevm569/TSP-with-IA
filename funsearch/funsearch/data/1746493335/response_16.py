import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with additional heuristics and stability."""

    # Initialize random number generator for reproducibility
    np.random.seed(42)

    # Use genetic algorithms to generate initial population of routes
    population = funsearch.genetic_algorithm(_distances)

    # Perform local search on each route to improve their quality
    population = funsearch.local_search(population, _distances)

    # Find the best route in the population
    best_route = funsearch.best_route(population)

    return best_route
