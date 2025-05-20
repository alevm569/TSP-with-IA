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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Simulated annealing
        - 2-opt

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the simulated annealing optimizer
    optimizer = funsearch.SimulatedAnnealing(
        initial_state=np.random.permutation(len(distances)),
        fitness_function=calculate_route_distance,
        distance_matrix=distances,
        temperature=1000,
        cooling_rate=0.99
    )

    # Run the optimizer for 1000 iterations
    best_route = optimizer.optimize(iterations=1000)

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
