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

    # Initialize the route
    route = list(range(len(_distances)))

    # Use the genetic algorithm to find the best route
    best_route, _ = funsearch.genetic_algorithm(
        route,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
        population_size=100,
        generations=100,
        crossover_rate=0.8,
        mutation_rate=0.2,
        seed=42,  # Set seed for reproducibility
    )

    return best_route
