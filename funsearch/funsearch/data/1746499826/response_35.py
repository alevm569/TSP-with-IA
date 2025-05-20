import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize population with random routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Evaluate each route in the population
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Perform genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness_values, generations=100)

    # Return the best route
    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
