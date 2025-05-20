import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic here.
    # For example, you could use a genetic algorithm or a hybrid approach.

    # Ensure reproducibility by setting a seed.
    np.random.seed(0)

    # Create a population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Evaluate each route in the population.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Select the best route.
    best_route = population[np.argmin(fitness_values)]

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
