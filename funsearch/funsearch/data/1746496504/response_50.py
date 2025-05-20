import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Hybrid heuristic combines nearest neighbor and 2-opt operations.
    """

    # Initialize a random route
    route = np.random.permutation(np.arange(len(distances)))

    # Use nearest neighbor to find a good starting point
    current_city = np.random.randint(len(distances))
    for _ in range(len(distances) - 1):
        nearest_city = np.argmin(distances[current_city])
        route = np.insert(route, 0, nearest_city)
        current_city = nearest_city

    # Use 2-opt to improve the route
    for i in range(len(distances)):
        for j in range(i + 2, len(distances)):
            distance_diff = distances[route[i]][route[j]] - distances[route[i]][route[(j - 1)]] - distances[route[(j - 2)]][route[j]] + distances[route[i]][route[(j - 2)]] + distances[route[(j - 1)]][route[j]]
            if distance_diff < 0:
                route = np.insert(route, j, route.pop(i))

    return route
