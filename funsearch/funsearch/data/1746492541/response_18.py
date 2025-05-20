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

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Generate a random initial route
    initial_route = np.random.permutation(np.arange(len(_distances)))

    # Perform local search using the 2-opt heuristic
    best_route = local_search(_distances, initial_route)

    return best_route

def local_search(_distances: np.ndarray, initial_route: np.ndarray) -> tuple[int, ...]:
    """
    Performs local search on the given route using the 2-opt heuristic.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)
    initial_route (np.ndarray): The initial route to search from

    Returns:
    tuple[int, ...]: The best route found through local search
    """

    best_route = initial_route.copy()
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                # Swap the two cities in the route
                new_route = best_route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is better, update the best route
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        # If no further improvements are found, break out of the loop
        if best_distance == calculate_route_distance(best_route, _distances):
            break

    return best_route
