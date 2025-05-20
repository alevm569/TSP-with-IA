import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the fitness function
    def fitness(route):
        total_distance = calculate_route_distance(route, _distances)
        return 1 / total_distance

    # Create a genetic algorithm optimizer
    optimizer = funsearch.GAOptimizer(fitness, _distances.shape[0])

    # Run the optimization
    best_route = optimizer.optimize()

    # Return the best route
    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Args:
        route: A permutation of cities.
        distances: A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
