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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid approach.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The best route as a permutation of city indices.
    """

    # Hybrid approach combining nearest neighbor and 2-opt heuristics
    # Initialize a random route
    np.random.seed(42)
    initial_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Use nearest neighbor to generate an initial tour
    current_city = initial_route[0]
    tour = [current_city]
    unvisited = set(initial_route[1:])

    while unvisited:
        nearest_city = min(unvisited, key=lambda city: matrix_distances[current_city][city])
        tour.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt swaps to improve the tour
    for i in range(len(tour)):
        for j in range(i + 1, len(tour)):
            distance_difference = (
                matrix_distances[tour[i]][tour[j]]
                + matrix_distances[tour[(j + 1) % len(tour)]][tour[(i + 1) % len(tour)]]
                - matrix_distances[tour[i]][tour[(j + 1) % len(tour)]]
                - matrix_distances[tour[(j + 1) % len(tour)]][tour[(i + 1) % len(tour)]]
            )
            if distance_difference < 0:
                tour[i], tour[j] = tour[j], tour[i]

    return tuple(tour)
