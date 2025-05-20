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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining the nearest neighbor and 2-opt heuristics.
    """

    # Initialize a random permutation of cities
    cities = np.random.permutation(np.arange(len(matrix_distances)))

    # Apply nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(matrix_distances))) - {current_city}

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Apply 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            if matrix_distances[route[i]][route[j]] < matrix_distances[route[i]][route[(j - 1)]] + matrix_distances[route[(j - 1)]][route[j]]:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
