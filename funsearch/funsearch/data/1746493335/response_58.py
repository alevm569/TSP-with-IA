import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach that combines local search with a genetic algorithm.
    """

    # Generate an initial population of routes using the genetic algorithm.
    population = funsearch.genetic_algorithm(_distances)

    # Perform local search on the best routes in the population.
    best_route = funsearch.local_search(population)

    # Return the best route found.
    return best_route
