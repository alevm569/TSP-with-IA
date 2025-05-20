import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of find_best_route_v2. Uses a hybrid approach combining nearest neighbor, 2-opt, and local search.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initial route using nearest neighbor
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Apply 2-opt local search to improve the route
    for _ in range(10):  # Number of iterations
        for i in range(1, len(route)):
            for j in range(i + 2, len(route)):
                if distances[route[i-1]][route[i]] + distances[route[j-1]][route[j]] < distances[route[i-1]][route[j]] + distances[route[i]][route[j-1]]:
                    route[i:j] = route[j-1:i-1:-1]

    return tuple(route)
