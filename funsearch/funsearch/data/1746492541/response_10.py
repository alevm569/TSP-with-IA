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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    The hybrid heuristic combines the nearest neighbor strategy for initial route construction
    and the 2-opt strategy for local search.
    """

    # Initial route using nearest neighbor
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Local search using 2-opt strategy
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_original = distances[route[i]][route[j]]
                distance_reversed = distances[route[i]][route[j - 1]] + distances[route[j]][route[i]] + distances[route[j - 1]][route[j]] - distance_original
                if distance_reversed < distance_original:
                    route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
