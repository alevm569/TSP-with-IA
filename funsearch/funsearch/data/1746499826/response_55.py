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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic of nearest neighbor and 2-opt.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initialize the route with the nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(start_city)

    while remaining_cities:
        current_city = route[-1]
        closest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(closest_city)
        remaining_cities.remove(closest_city)

    # Use the 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = _distances[route[i]][route[j]]
            distance_reversed = _distances[route[i]][route[j - 1]] + _distances[route[j]][route[i + 1]] - distance_original
            if distance_reversed < distance_original:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
