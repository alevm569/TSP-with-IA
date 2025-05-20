import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
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
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Genetic Algorithm with Tournament Selection and Elitism

    Routes must include all cities exactly once and return to the starting point.
    """

    # Create a genetic algorithm object
    ga = funsearch.GA(distances, pop_size=100, max_iter=100)

    # Set tournament selection with elitism
    ga.tournament_selection = True
    ga.elitism = True

    # Run the genetic algorithm
    best_route = ga.run()

    return best_route
