import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to search for the best route.
    # Define the fitness function.
    def fitness(route):
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
        return total_distance

    # Create the genetic algorithm object.
    ga = funsearch.GA(fitness)

    # Run the genetic algorithm.
    best_route = ga.run()

    # Return the best route.
    return best_route
