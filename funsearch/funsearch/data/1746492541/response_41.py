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
    Improved version of find_best_route_v2 using a hybrid heuristic combining nearest neighbor and 2-opt.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Initial solution using nearest neighbor
    start_city = np.random.randint(len(matrix_distances))
    route = [start_city]
    remaining_cities = set(range(len(matrix_distances)))
    remaining_cities.remove(start_city)

    while remaining_cities:
        current_city = route[-1]
        closest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(closest_city)
        remaining_cities.remove(closest_city)

    # Local search using 2-opt
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
