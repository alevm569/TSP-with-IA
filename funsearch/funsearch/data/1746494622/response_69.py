import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function combines the nearest neighbor heuristic for initialization and the 2-opt local search heuristic for optimization.
    """

    # Initialize a random route
    route = np.random.permutation(np.arange(len(_distances)))

    # Perform nearest neighbor heuristic to improve initial route
    current_city = route[0]
    for i in range(1, len(route)):
        nearest_city = None
        min_distance = float('inf')
        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    nearest_city = city
                    min_distance = distance
        route[i] = nearest_city
        current_city = nearest_city

    # Perform 2-opt local search to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route.copy()
            new_route[i:j+1] = new_route[j:i:-1]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
