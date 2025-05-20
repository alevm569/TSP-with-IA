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

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Use a combination of nearest neighbor and 2-opt heuristics
    # Initialize a random route
    initial_route = np.random.permutation(np.arange(len(_distances)))

    # Use nearest neighbor to fill in missing cities
    for i in range(len(_distances)):
        if i not in initial_route:
            nearest_city = np.argmin(_distances[initial_route[-1]])
            initial_route = np.append(initial_route, nearest_city)

    # Use 2-opt to improve the route
    for i in range(100):
        for j in range(len(_distances)):
            for k in range(j + 1, len(_distances)):
                new_route = np.delete(initial_route, [j, k])
                new_route = np.insert(new_route, j, k)
                new_route = np.insert(new_route, k, j)
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(initial_route, _distances):
                    initial_route = new_route

    return tuple(initial_route)
