import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Initialize population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define fitness function to minimize total route distance
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Perform genetic algorithm optimization
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
