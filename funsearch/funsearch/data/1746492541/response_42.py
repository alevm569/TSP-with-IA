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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of the find_best_route function using a hybrid approach.

    Combines the nearest neighbor heuristic for initialization, the 2-opt local search for optimization,
    and a tabu search to avoid getting stuck in local minima.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Optimize route using 2-opt local search
    funsearch.tabu_search(route, distances, max_iter=100)

    return route
