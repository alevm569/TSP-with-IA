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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v0`.

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial route using nearest neighbor heuristic
    current_city = np.random.randint(len(distances))
    route = [current_city]
    remaining_cities = set(range(len(distances))) - {current_city}

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Optimize the route using 2-opt heuristic
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_diff = distances[route[i]][route[j]] - distances[route[(i + 1) % len(route)]][route[(j - 1) % len(route)]]
                if distance_diff < 0:
                    route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
