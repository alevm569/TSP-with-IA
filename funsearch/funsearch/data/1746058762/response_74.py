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

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """

    # Perform local search optimization using the 2-opt heuristic.
    best_route = np.random.permutation(len(matrix_distances))
    current_route = best_route.copy()

    while True:
        # Find the two best edges to swap in the route.
        best_delta = float('inf')
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                delta = matrix_distances[current_route[i]][current_route[(j + 1) % len(current_route)]] \
                       + matrix_distances[current_route[j]][current_route[(i + 1) % len(current_route)]] \
                       - matrix_distances[current_route[i]][current_route[j]]

                if delta < best_delta:
                    best_delta = delta
                    best_i, best_j = i, j

        # If no improvements were found, break the loop.
        if best_delta == float('inf'):
            break

        # Swap the two best edges.
        current_route[best_i], current_route[best_j] = current_route[best_j], current_route[best_i]

    return tuple(current_route)
