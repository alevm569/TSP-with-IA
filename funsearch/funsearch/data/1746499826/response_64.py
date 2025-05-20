import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Initialize the best route and distance
    best_route = None
    best_distance = float('inf')

    # Generate a set of candidate routes using the nearest neighbor heuristic
    routes = funsearch.nearest_neighbor(_distances)

    # Iterate over the candidate routes and find the best one
    for route in routes:
        distance = calculate_route_distance(route, _distances)
        if distance < best_distance:
            best_distance = distance
            best_route = route

    return best_route

def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
