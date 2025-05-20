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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Nearest neighbor
        - 2-opt

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random route
    num_cities = len(matrix_distances)
    route = np.random.permutation(num_cities)

    # Nearest neighbor heuristic
    for i in range(num_cities):
        current_city = route[i]
        next_city = -1
        min_distance = float('inf')

        for j in range(num_cities):
            if j != current_city and matrix_distances[current_city][j] < min_distance:
                next_city = j
                min_distance = matrix_distances[current_city][j]

        route[i+1] = next_city

    # 2-opt heuristic
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 2, num_cities):
                distance_original = matrix_distances[route[i]][route[j]]
                distance_reversed = matrix_distances[route[i]][route[(j-1)%num_cities]] + matrix_distances[route[j]][route[(i+1)%num_cities]]
                if distance_reversed < distance_original:
                    route[i+1:j] = route[i+1:j][::-1]

    return route
