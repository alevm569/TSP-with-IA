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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v1, using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristic combination:
        - Nearest neighbor for initial route construction.
        - Cheapest insertion for subsequent route construction.
        - Local search with k-opt and 2-opt operations.

    Routes must include all cities exactly once and return to the starting point.
    """
    # Initial route using nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    visited = np.zeros(len(distances), dtype=bool)
    visited[start_city] = True

    for _ in range(len(distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin(~visited, axis=0)[current_city]
        route.append(nearest_city)
        visited[nearest_city] = True

    # Subsequent route construction using cheapest insertion heuristic
    for _ in range(len(distances) - 1):
        current_city = route[-1]
        cheapest_city = np.argmin(distances[current_city][~visited])
        route.append(cheapest_city)
        visited[cheapest_city] = True

    # Local search using k-opt and 2-opt operations
    funsearch.k_opt(route, distances)
    funsearch.two_opt(route, distances)

    return tuple(route)
