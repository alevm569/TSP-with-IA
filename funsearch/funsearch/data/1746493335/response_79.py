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

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use ACO heuristic to initialize route
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities, _distances)
    best_route = aco.find_best_route()

    # Apply 2-opt heuristic to improve route quality
    for i in range(len(best_route)):
        for j in range(i + 2, len(best_route)):
            distance_before = _distances[best_route[i]][best_route[j]]
            best_route[i+1:j] = reversed(best_route[i+1:j])
            distance_after = _distances[best_route[i]][best_route[j]]
            if distance_after < distance_before:
                break

    return tuple(best_route)
