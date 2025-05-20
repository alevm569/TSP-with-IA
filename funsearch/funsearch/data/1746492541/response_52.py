import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use a combination of the nearest neighbor and cheapest insertion heuristics
    best_route = funsearch.nearest_neighbor(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    for _ in range(100):
        candidate_route = funsearch.cheapest_insertion(best_route, _distances)
        candidate_distance = calculate_route_distance(candidate_route, _distances)

        if candidate_distance < best_distance:
            best_route = candidate_route
            best_distance = candidate_distance

    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        j = (i + 1) % len(route)
        total_distance += distances[route[i]][route[j]]
    return total_distance
