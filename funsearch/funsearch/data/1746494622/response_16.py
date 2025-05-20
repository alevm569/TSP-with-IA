import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of find_best_route_v2."""

    # Use a genetic algorithm to find the best route
    population_size = 50
    generations = 50
    tournament_size = 5

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(distances, population_size, generations, tournament_size)

    # Run the genetic algorithm
    best_route = ga.run()

    return best_route
