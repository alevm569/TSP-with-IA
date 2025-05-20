import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def calculate_route_distance(route: tuple[int], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        j = (i + 1) % len(route)
        total_distance += distances[route[i]][route[j]]
    return total_distance

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a population of candidate routes
    population = np.random.permutation(len(_distances))

    # Define the fitness function
    def fitness(route: tuple[int]) -> float:
        return 1 / calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness)

    return best_route
