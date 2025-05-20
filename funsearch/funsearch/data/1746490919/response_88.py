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

    Use a hybrid heuristic combining 2-opt and local search with a random restart mechanism.
    """

    # Generate an initial random route
    np.random.seed(0)
    initial_route = np.random.permutation(len(matrix_distances))

    # Perform hybrid heuristic with local search and 2-opt
    best_route = funsearch.hybrid_heuristic(
        initial_route,
        [funsearch.local_search, funsearch.two_opt],
        max_iterations=1000,
        random_restarts=5,
        distance_matrix=matrix_distances,
    )

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
