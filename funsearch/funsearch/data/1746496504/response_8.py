import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristics."""

    # Initialize population with random routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define fitness function for route evaluation
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm with elitism and mutation
    best_route = funsearch.genetic_algorithm(population, fitness, elitism=True, mutation_probability=0.1)

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
