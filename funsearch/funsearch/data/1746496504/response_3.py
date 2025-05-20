import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(0)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    # Define the fitness function
    def fitness(route: np.ndarray) -> float:
        return -calculate_route_distance(route, _distances)

    # Create the genetic algorithm
    ga = funsearch.GA(fitness, _distances.shape[0])

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route
    return best_route

# Helper function to calculate the total distance of a route
def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
