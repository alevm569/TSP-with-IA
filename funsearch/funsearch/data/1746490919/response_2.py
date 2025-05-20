import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Apply a hybrid approach combining two heuristics:
    # 1. Nearest neighbor heuristic to generate an initial tour.
    # 2. 2-opt heuristic to improve the tour by swapping two edges.

    # Generate an initial tour using nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    tour = [current_city]

    while len(tour) < len(_distances):
        next_city = np.argmin(_distances[current_city])
        if next_city not in tour:
            tour.append(next_city)
            current_city = next_city

    # Apply 2-opt heuristic to improve the tour
    for i in range(len(tour)):
        for j in range(i + 1, len(tour)):
            new_tour = tour[:]
            new_tour[i], new_tour[j] = new_tour[j], new_tour[i]
            if calculate_route_distance(new_tour, _distances) < calculate_route_distance(tour, _distances):
                tour = new_tour

    return tuple(tour)
