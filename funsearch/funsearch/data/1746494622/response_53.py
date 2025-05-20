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

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Implement your new heuristic here, for example:
    # 1. Implement a hybrid heuristic that combines different heuristics.
    # 2. Implement a novel heuristic that uses a different approach to find the best route.

    # Example hybrid heuristic:
    # 1. Use the nearest neighbor heuristic to find an initial route.
    # 2. Use the 2-opt heuristic to iteratively improve the route.
    # 3. Use the local search heuristic to further refine the route.

    # Example novel heuristic:
    # 1. Use a genetic algorithm to search for the best route.
    # 2. Use a reinforcement learning approach to guide the search.

    # Return the best route as a tuple of city indices.
    return tuple(np.random.permutation(np.arange(len(matrix_distances))))


def find_best_route_v0(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route`."""
    return find_best_route(_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    return find_best_route(_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    return find_best_route(_distances)
