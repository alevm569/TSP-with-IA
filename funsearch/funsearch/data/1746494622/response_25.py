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

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Initialize random permutation of cities
    cities = np.random.permutation(len(_distances))

    # Nearest neighbor heuristic
    current_city = cities[0]
    route = [current_city]
    unvisited = set(range(len(_distances))) - {current_city}

    while unvisited:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Add return to starting city
    route.append(route[0])

    return tuple(route)
