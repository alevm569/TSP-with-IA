import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



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
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining nearest neighbor and k-opt heuristics.
    """

    # Initialize a random permutation of cities
    np.random.seed(42)  # Set a seed for reproducibility
    current_route = np.random.permutation(len(distances))

    # Nearest neighbor heuristic
    for i in range(len(distances)):
        current_city = current_route[i]
        closest_city = np.argmin(distances[current_city])
        current_route[i] = closest_city

    # K-opt heuristic
    for k in range(2, len(distances)):
        for i in range(len(distances)):
            best_distance = math.inf
            for j in range(i + 2, len(distances)):
                distance = calculate_route_distance(current_route[i:j+1], distances)
                if distance < best_distance:
                    best_distance = distance
                    best_route = current_route[i:j+1]

            current_route[i:j+1] = best_route

    return current_route


def calculate_route_distance(route: tuple[int], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
