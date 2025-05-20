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
    Improved version of `find_best_route_v2` using a hybrid approach.

    This function uses a combination of the nearest neighbor heuristic and the 2-opt local search algorithm.
    """

    # Initialize a random permutation of cities
    np.random.seed(42)
    initial_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Use nearest neighbor to find an initial tour
    current_city = initial_route[0]
    tour = [current_city]

    while len(tour) < len(matrix_distances):
        closest_city = np.argmin(matrix_distances[current_city])
        if closest_city not in tour:
            tour.append(closest_city)
            current_city = closest_city

    # Use 2-opt local search to refine the tour
    def two_opt(tour: list[int]) -> list[int]:
        best_distance = calculate_route_distance(tour, matrix_distances)

        for i in range(len(tour)):
            for j in range(i + 1, len(tour)):
                new_tour = tour[:i] + tour[j:i:-1] + tour[j + 1:]
                new_distance = calculate_route_distance(new_tour, matrix_distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    tour = new_tour

        return tour

    for _ in range(10):
        tour = two_opt(tour)

    return tuple(tour)


def calculate_route_distance(route: tuple[int], matrix_distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0

    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
