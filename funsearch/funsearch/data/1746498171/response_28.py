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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Strategy:
        - Local Search with Simulated Annealing

    Routes must include all cities exactly once and return to the starting point.
    """

    # Generate an initial random route
    np.random.seed(42)  # Set seed for reproducibility
    initial_route = np.random.permutation(len(matrix_distances))

    # Initialize the local search algorithm
    ls = funsearch.LocalSearch(matrix_distances, initial_route)

    # Run simulated annealing with a temperature schedule
    best_route = ls.simulated_annealing(temperature=10000, cooling_rate=0.99)

    return best_route
