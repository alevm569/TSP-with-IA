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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a combination of the nearest neighbor and 2-opt heuristics.

    Parameters:
        matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A tuple representing the best route.
    """

    # Nearest neighbor heuristic
    num_cities = len(matrix_distances)
    current_city = 0
    route = [current_city]

    while len(route) < num_cities:
        closest_city = -1
        min_distance = float('inf')

        for i in range(num_cities):
            if i not in route:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    closest_city = i

        route.append(closest_city)
        current_city = closest_city

    # 2-opt heuristic
    for _ in range(20):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                distance_before = matrix_distances[route[i]][route[j]]
                distance_after = matrix_distances[route[i]][route[(j + 1) % num_cities]] + \
                                matrix_distances[route[j]][route[(i + 1) % num_cities]]
                if distance_after < distance_before:
                    route[i], route[j] = route[j], route[i]

    return tuple(route)
